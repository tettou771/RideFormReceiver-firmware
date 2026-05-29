/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "hid.h"
#include "tdma.h"
#include "tdma_proto.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/crc.h>

#include "esb.h"
#include "cmd_queue.h"
#include "timer.h"
#include "modem.h"

static struct esb_payload rx_payload;
//static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);
//static struct esb_payload tx_payload_timer = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
// Sync packet extended to 12 bytes: LED clock (2B) + RFT command slot (10B).
// See cmd_queue.h for packet layout.
static struct esb_payload tx_payload_sync = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

uint8_t pairing_buf[8] = {0};
static uint8_t discovered_trackers[MAX_TRACKERS] = {0};

/* RFT diagnostic: per-tracker packet RX counts. Reset each second by the
 * stats logger so the printout reflects pps. Helps tell whether a
 * "disconnected" tracker (PC sees no packets) is actually missing at the
 * radio level or just not reaching HID. */
static volatile uint32_t rft_rx_count[MAX_TRACKERS] = {0};
static volatile uint32_t rft_rx_pretrig[MAX_TRACKERS] = {0};  /* pre-discovery filter drops */
static volatile uint32_t rft_hid_count[MAX_TRACKERS] = {0};   /* successful HID forwards */
uint8_t sequences[256] = {0};
int64_t last_seq_time[256] = {[0 ... 255] = -1000};
uint16_t packets_count[256] = {0};
uint8_t packets_lost[256] = {0};

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_packet_filter_thread(void);
K_THREAD_DEFINE(esb_packet_filter_thread_id, 256, esb_packet_filter_thread, NULL, NULL, NULL, 6, 0, 0);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, 6, 0, 0);

static void esb_parse_pair(void);

//|type    |description
//|RX  CRC8|pairing
//|TX  CRC8|pairing

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |data                                                                                                                                  |
//|RX  CRC8|ack     |device_addr                                          |-
//|TX  CRC8|ack     |recv_addr                                            |-

// TDMA to implement

//|packet  |description
//|RX     1|request from tracker
//|TX     2|pairing accepted from dongle
//|TX     3|Dongle State
//|TX     4|No Windows
//|TX     5|Window Info

//|packet  |b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|RX     1|    0xCD|    0x01|    0x00|Tracker Hardware ID                                  |Tracker Hardware ID                                  |-
//|TX     2|    0xCD|    0x02|Trckr ID|Dongle Hardware ID                                   |Tracker Hardware ID                                  |-
//|TX     3|    0xCD|    0x03|Dongle Hardware ID                                   |state   |channel |-
//|TX     4|    0xCD|    0x04|-
//|TX     5|    0xCD|    0x05|Window  |Timer                              |Packet  |-

//packet 3:
//state field bits: 9[0:0]: Accepts new trackers?; 9[1:1]: Force pair
//channel bundle field bits: 10[0:3]: Channels bundle; 10[4:7]: Next channel offset
//packet 5:
//Packet: Packet Number

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED");
		break;
	case ESB_EVENT_RX_RECEIVED:
		LOG_DBG("RX");
	// TODO: make tx payload for ack here
		int err = 0;
		while (!err) // zero, rx success
		{
			err = esb_read_rx_payload(&rx_payload);
			if (err == -ENODATA)
			{
				return;
			}
			else if (err)
			{
				LOG_ERR("Error while reading rx packet: %d", err);
				return;
			}
			// TODO: split into separate handlers
			switch (rx_payload.pipe)
			{
			case 0: // base address 0 (pairing address)
				if (rx_payload.length != 8)
				{
					LOG_ERR("Wrong packet length: %d", rx_payload.length);
					continue;
				}
				LOG_DBG("rx: %16llX", *(uint64_t *)rx_payload.data);
				memcpy(pairing_buf, rx_payload.data, 8);
				switch (pairing_buf[1])
				{
				case 1: // receives ack generated from last packet
					LOG_DBG("RX Pairing Sent ACK");
					break;
				case 2: // should "acknowledge" pairing data sent from receiver
					LOG_DBG("RX Pairing ACK Receiver");
					break;
				default: // first packet in pairing burst
					LOG_INF("RX Pairing Request");
					break;
				}
				continue;
			default: // base address 1
			}
			switch (rx_payload.length)
			{
			case 21: // has sequence number
				// TODO : It's a very crude implementation
				// But brain hurty, will make a better one later
				uint8_t seq = rx_payload.data[20];
				uint8_t tracker_id = rx_payload.data[1];
				uint8_t next = sequences[tracker_id] + 1; // wrap
				if(seq != 0 && sequences[tracker_id] != 0 && next != seq) {
					if (k_uptime_get() - last_seq_time[tracker_id] < 100 && ((next < 128) // reset sequence if last packet was over 100ms old
						? ((seq < next) || (seq >= next + 128)) // next 0-127: seq is below next or above or equal to next +128
						: ((seq < next) && (seq >= next - 128)))) // next 128-255: seq is below next and above or equal to next -128
					{
						LOG_WRN("Sequence missmatch for tracker %d, expected %d, got %d. Discarding.", tracker_id, next, seq);
						break;
					}
				}
				sequences[tracker_id] = seq;
				last_seq_time[tracker_id] = k_uptime_get();
				// Fall-throught
			case 20: // has crc32
				uint32_t crc_check = crc32_k_4_2_update(0x93a409eb, rx_payload.data, 16);
				uint32_t *crc_ptr = (uint32_t *)&rx_payload.data[16];
				if (*crc_ptr != crc_check)
				{
					LOG_ERR("Incorrect checksum, computed %08X, received %08X", crc_check, *crc_ptr);
					printk("%08llx%016llX%016llX\n", *(uint64_t *)&rx_payload.data[16] & 0XFFFFFFFF, *(uint64_t *)&rx_payload.data[8], *(uint64_t *)rx_payload.data);
					break;
				}
				// Fall-throught
			case 16:
				uint8_t imu_id = rx_payload.data[1];
				if (imu_id >= stored_trackers) // not a stored tracker
					continue;
				if (imu_id < MAX_TRACKERS) rft_rx_count[imu_id]++;
				if (discovered_trackers[imu_id] < DETECTION_THRESHOLD) // garbage filtering of nonexistent tracker
				{
					discovered_trackers[imu_id]++;
					if (imu_id < MAX_TRACKERS) rft_rx_pretrig[imu_id]++;
					continue;
				}
				if (rx_payload.data[0] > 223) // reserved for receiver only
					break;
				hid_write_packet_n(rx_payload.data, rx_payload.rssi); // write to hid endpoint
				modem_enqueue_tracker_packet(rx_payload.data); // forward over LTE (no-op on USB-only boards)
				if (imu_id < MAX_TRACKERS) rft_hid_count[imu_id]++;
				break;
			default:
				LOG_ERR("Wrong packet length: %d", rx_payload.length);
				break;
			}
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10000) {
			LOG_WRN("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

// this was randomly generated
// TODO: I have no idea?
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8}; // TODO: not used
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

// RFT: not static — sync_thread in timer.c reads this to gate its PRX↔PTX swap.
bool esb_initialized = false;

int esb_initialize(bool tx)
{
	// RFT: sync_thread reconfigures (PRX↔PTX) on every cycle; the upstream
	// esb_disable() (nrf ESB lib) does not clear our esb_initialized flag,
	// so this used to flood "ESB already initialized" warnings. Just clear
	// it here — esb_init() below reconfigures the radio either way.
	esb_initialized = false;
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 30;
		// config.retransmit_delay = 600;
		config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
//		config.use_fast_ramp_up = true;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 30;
		// config.retransmit_delay = 600;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
//		config.use_fast_ramp_up = true;
	}

	// RFT: was LOG_INF — at 20 Hz this floods the console. Bumped to DBG.
	LOG_DBG("Initializing ESB, %sX mode", tx ? "T" : "R");
	err = esb_init(&config);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (err)
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	esb_initialized = true;
	return 0;
}

static void esb_deinitialize(void)
{
	LOG_INF("ESB deinitialize requested");
	if (esb_initialized)
	{
		esb_initialized = false;
		LOG_INF("Deinitializing ESB");
		k_msleep(10); // wait for pending transmissions
		if (esb_initialized)
		{
			LOG_INF("ESB denitialize cancelled");
			return;
		}
		esb_disable();
	}
	esb_initialized = false;
}

// TODO: not used
inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Generate addresses from device address
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, addr, 6);
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = buf[i];
		addr_buffer[i + 4] = buf[i] + buf[4];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = buf[5] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
//	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
//	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

static bool esb_pairing = false;
// RFT: not static — sync_thread in timer.c reads this.
bool esb_paired = false;

void esb_add_pair(uint64_t addr, bool checksum)
{
	int id = stored_trackers;
	if (checksum)
	{
		for (int i = 0; i < stored_trackers; i++) // Check if the device is already stored
		{
			if (addr != 0 && stored_tracker_addr[i] == addr)
			{
				id = i;
			}
		}
	}
	if (id == stored_trackers)
	{
		LOG_INF("Added device on id %d with address %012llX", id, addr);
		stored_tracker_addr[id] = addr;
		sys_write(STORED_ADDR_0 + id, NULL, &stored_tracker_addr[id], sizeof(stored_tracker_addr[0]));
		stored_trackers++;
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));

		/* RFT TDMA: queue SET_SLOT_INDEX so the new tracker learns its
		 * slot position immediately, without needing a console
		 * `broadcast_index`. Only meaningful for the first TDMA_NUM_SLOTS
		 * trackers; anything beyond is paired but unschedulable. */
		if (id < TDMA_NUM_SLOTS) {
			rft_cmd_t cmd = { .type = RFT_CMD_SET_SLOT_INDEX };
			cmd.data[0] = (uint8_t)id;
			(void)rft_cmd_push((uint8_t)id, &cmd);
			LOG_INF("TDMA: queued SET_SLOT_INDEX=%d for new tracker", id);
		}
	}
	else
	{
		LOG_INF("Device already stored with id %d", id);
	}
	if (checksum)
	{
		uint8_t buf[6] = {0};
		memcpy(buf, &addr, 6);
		uint8_t checksum = crc8_ccitt(0x07, buf, 6);
		if (checksum == 0)
			checksum = 8;
		uint64_t *receiver_addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet
		addr = (*receiver_addr & 0xFFFFFFFFFFFF) << 16;
		addr |= checksum; // Add checksum to the address
		addr |= (uint64_t)id << 8; // Add tracker id to the address
		LOG_INF("Pair the device with %016llX", addr);
	}
}

void esb_pop_pair(void)
{
	if (stored_trackers > 0)
	{
		stored_trackers--;
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
		LOG_INF("Removed device on id %d with address %012llX", stored_trackers, stored_tracker_addr[stored_trackers]);
	}
	else
	{
		LOG_WRN("No devices to remove");
	}
}

void esb_parse_pair()
{
	uint64_t found_addr = (*(uint64_t *)pairing_buf >> 16) & 0xFFFFFFFFFFFF;
	uint16_t send_tracker_id = stored_trackers; // Use new tracker id
	for (int i = 0; i < stored_trackers; i++) // Check if the device is already stored
	{
		if (found_addr != 0 && stored_tracker_addr[i] == found_addr)
		{
			//LOG_INF("Found device linked to id %d with address %012llX", i, found_addr);
			send_tracker_id = i;
		}
	}
	uint8_t checksum = crc8_ccitt(0x07, &pairing_buf[2], 6); // make sure the packet is valid
	if (checksum == 0)
		checksum = 8;
	if (checksum == pairing_buf[0] && found_addr != 0 && send_tracker_id == stored_trackers && stored_trackers < MAX_TRACKERS) // New device, add to NVS
	{
		esb_add_pair(found_addr, false);
		set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
	}
	if (checksum == pairing_buf[0] && send_tracker_id < MAX_TRACKERS) // Make sure the dongle is not full
		tx_payload_pair.data[0] = pairing_buf[0]; // Use checksum sent from device to make sure packet is for that device
	else
		tx_payload_pair.data[0] = 0; // Invalidate packet
	tx_payload_pair.data[1] = send_tracker_id; // Add tracker id to packet
}

void esb_pair(void)
{
	LOG_INF("Pairing");
	esb_set_addr_paired();
	esb_initialize(false);
	esb_start_rx();
	tx_payload_pair.pipe = 0;
	tx_payload_pair.noack = false;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	memcpy(&tx_payload_pair.data[2], addr, 6);
	LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	esb_pairing = true;
	pairing_buf[1] = 255; // initialize packet flag
	while (esb_pairing)
	{
		if (!esb_initialized)
		{
			esb_initialize(false);
			esb_start_rx();
		}
		switch (pairing_buf[1])
		{
		case 0: // first packet in pairing burst
			esb_parse_pair();
			LOG_DBG("tx: %16llX", *(uint64_t *)tx_payload_pair.data);
//			esb_flush_tx();
			esb_write_payload(&tx_payload_pair); // Add to TX buffer
			pairing_buf[1] = 255; // flag packet processed
			k_msleep(10);
			esb_flush_tx(); // Flush TX buffer for next pairing burst
			continue;
		case 2:
			esb_flush_tx(); // Flush TX buffer for next pairing burst
		case 255:
		default:
			break;
		}
		pairing_buf[1] = 255; // flag packet processed
		//esb_flush_rx();
		//esb_flush_tx();
		//esb_write_payload(&tx_payload_pair); // Add to TX buffer
		k_usleep(1);
	}
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CONNECTION);
	esb_deinitialize();
}

void esb_reset_pair(void)
{
	esb_deinitialize(); // make sure esb is off
	esb_paired = false;
}

void esb_finish_pair(void)
{
	esb_pairing = false;
}

void esb_clear(void)
{
	stored_trackers = 0;
	sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	LOG_INF("NVS Reset");
	esb_reset_pair();
}

// RFT: queue a TDMA TIMING ACK as the next ACK payload. Same delivery
// mechanism as esb_write_sync below — ESB hardware auto-attaches the
// queued payload to the next tracker's outgoing ACK. Trackers branch on
// data[0]=0xFB to parse this as TDMA timing instead of a CMD ACK.
//
// See tdma.c for the cycle clock and us_until_next_cycle computation.
void esb_write_tdma_timing(void)
{
	if (!esb_initialized || !esb_paired)
		return;
	tx_payload_sync.pipe = 1;
	tx_payload_sync.noack = false;
	tx_payload_sync.length = 12;
	tdma_pack_timing_ack(tx_payload_sync.data);
	int err = esb_write_payload(&tx_payload_sync);
	(void)err;  /* -ENOMEM is expected when FIFO is still hot from previous write */
}

// RFT: queue the next sync as an ACK payload.
//
// Receiver stays in PRX. ESB's auto-ack engine attaches the queued payload
// to the next outgoing ACK frame on a *matching pipe*. All trackers send on
// pipe 1 (see tracker firmware's connection.c — tx_payload.pipe = 1 with
// tracker_id encoded in data[1]), so we queue ACK payloads on pipe 1.
//
// CONFIG_ESB_TX_FIFO_SIZE = 1, so this FIFO holds a single pending entry.
// If we call faster than trackers transmit, we get -ENOMEM until the next
// tracker packet drains it. That's expected and harmless. Counts of
// success vs ENOMEM are surfaced in the 1-Hz diagnostic.
void esb_write_sync(uint16_t led_clock)
{
	if (!esb_initialized || !esb_paired)
		return;
	tx_payload_sync.pipe = 1; // all trackers TX on pipe 1
	tx_payload_sync.noack = false;
	tx_payload_sync.length = 12;
	tx_payload_sync.data[0] = (led_clock >> 8) & 255;
	tx_payload_sync.data[1] = led_clock & 255;
	rft_cmd_t cmd;
	uint8_t target = rft_cmd_peek_next(&cmd);
	uint8_t flags = rft_get_global_flags();
	rft_cmd_pack(flags, target, target == RFT_CMD_NO_TARGET ? NULL : &cmd,
	             &tx_payload_sync.data[2]);
	int err = esb_write_payload(&tx_payload_sync);

	// Only consume (decrement retry counter / drop) on successful queue.
	// If -ENOMEM, the ACK FIFO is still full from the previous write —
	// next tick we'll peek the same cmd and try again.
	if (err == 0 && target != RFT_CMD_NO_TARGET) {
		rft_cmd_consume(target);
	}

	// Diag: success / busy / cmd-issued counts.
	static volatile uint32_t ok_count = 0;
	static volatile uint32_t busy_count = 0;
	static volatile uint32_t cmd_tx_count = 0;
	static int64_t last_log_ms = 0;
	if (err == 0) ok_count++;
	else if (err == -ENOMEM) busy_count++;
	if (err == 0 && target != RFT_CMD_NO_TARGET) cmd_tx_count++;
	int64_t now_ms = k_uptime_get();
	if (now_ms - last_log_ms > 1000) {
		last_log_ms = now_ms;
		printk("RFT_RX: ack_ok/s=%u busy/s=%u cmd_tx/s=%u last_err=%d paired=%d init=%d\n",
		       ok_count, busy_count, cmd_tx_count, err,
		       (int)esb_paired, (int)esb_initialized);
		ok_count = 0;
		busy_count = 0;
		cmd_tx_count = 0;

		/* Per-tracker RX / HID-forward counts (pps). Helps tell which
		 * tracker_id is silent at the radio level vs. silent only on the
		 * HID side. Print only if at least one tracker is paired so
		 * unpaired-receiver runs stay quiet. */
		if (stored_trackers > 0) {
			char buf[256];
			int p = 0;
			p += snprintf(buf + p, sizeof(buf) - p, "RFT_RX_PER:");
			for (int i = 0; i < stored_trackers && p < (int)sizeof(buf) - 16; i++) {
				p += snprintf(buf + p, sizeof(buf) - p,
				              " t%d=%u/%u", i,
				              (unsigned)rft_hid_count[i],
				              (unsigned)rft_rx_count[i]);
			}
			printk("%s\n", buf);
			for (int i = 0; i < MAX_TRACKERS; i++) {
				rft_rx_count[i] = 0;
				rft_hid_count[i] = 0;
				rft_rx_pretrig[i] = 0;
			}
		}
	}
}

// TODO:
void esb_receive(void)
{
	esb_set_addr_paired();
	esb_paired = true;
}

static void esb_packet_filter_thread(void)
{
	memset(discovered_trackers, 0, sizeof(discovered_trackers));
	while (1) // reset count if its not above threshold
	{
		k_msleep(1000);
		for (int i = 0; i < MAX_TRACKERS; i++)
			if (discovered_trackers[i] < DETECTION_THRESHOLD)
				discovered_trackers[i] = 0;
	}
}

static void esb_thread(void)
{
	clocks_start();

	sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
	if (stored_trackers)
		esb_paired = true;
	for (int i = 0; i < stored_trackers; i++)
		sys_read(STORED_ADDR_0 + i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	LOG_INF("%d/%d devices stored", stored_trackers, MAX_TRACKERS);

	if (esb_paired)
	{
		esb_receive();
		esb_initialize(false);
		esb_start_rx();
	}

	// RFT: kick off the periodic sync timer. Drives esb_write_sync which
	// delivers RFT_CMD commands queued via cmd_queue. Disabled for now —
	// nrfx_timer setup needs more work (boot crashes), revisit with k_work.
	// timer_init();

	while (1)
	{
		if (!esb_paired)
		{
			esb_pair();
			esb_receive();
			esb_initialize(false);
			esb_start_rx();
		}
		k_msleep(100);
	}
}
