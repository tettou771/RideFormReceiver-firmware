/*
 * RFT: BG770A-GL modem driver (skeleton).
 *
 * Drives the Quectel BG770A-GL on UART1 with raw AT commands. The Zephyr
 * modem subsystem (drivers/modem) doesn't speak BG770A's CMUX dialect, so
 * we walk the state machine ourselves.
 *
 * Boot sequence (real device, to be validated on hardware):
 *   PWRKEY pulse 600 ms -> wait "RDY" URC -> ATE0 -> AT+CMEE=1 ->
 *   AT+CPIN? -> AT+CFUN=1 -> AT+CEREG=2 wait registered ->
 *   AT+QICSGP / AT+QIACT (PDP up) -> AT+QMTOPEN broker ->
 *   AT+QMTCONN clientid -> READY.
 *
 * Hot path (READY): aggregate 16B tracker packets in a ring; flush at
 * 30 Hz with one AT+QMTPUB (binary mode). 16B * ~8 trackers * 30 Hz ~=
 * 4 kbps payload, well within LTE-M.
 *
 * SPDX-License-Identifier: MIT
 */
#include "modem.h"

#include "globals.h"
#include "system/system.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(modem, LOG_LEVEL_INF);

#define MDM_USER_NODE  DT_PATH(zephyr_user)
#define MDM_AVAILABLE  DT_NODE_HAS_PROP(MDM_USER_NODE, modem_power_gpios)

#if !MDM_AVAILABLE

/* Stubs for boards without the BG770A-GL modem (e.g. xiao_ble). esb.c
 * still references the public API; we keep the symbols around as no-ops
 * so all build targets link. */

void modem_request_start(void) {}
void modem_request_stop(void) {}
const char *modem_state_string(void) { return "unavailable"; }
bool modem_enqueue_tracker_packet(const uint8_t *pkt16) { (void)pkt16; return true; }
void modem_console_status(void) {}
void modem_console_at(const char *cmd) { (void)cmd; }
void modem_console_set_apn(const char *s) { (void)s; }
void modem_console_set_broker(const char *host, uint16_t port) { (void)host; (void)port; }
void modem_console_set_client(const char *s) { (void)s; }
void modem_console_set_topic(const char *s) { (void)s; }
void modem_console_set_user(const char *s) { (void)s; }
void modem_console_set_pass(const char *s) { (void)s; }
void modem_set_pace(int ms) { (void)ms; }
void modem_set_ptmo(int ms) { (void)ms; }
int  modem_get_pace(void) { return 0; }
int  modem_get_ptmo(void) { return 0; }

#else /* MDM_AVAILABLE */

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/reboot.h>

/* ---- DT bindings ----------------------------------------------------- */

#define UART_NODE DT_NODELABEL(uart1)
static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
static const struct gpio_dt_spec mdm_pwr =
	GPIO_DT_SPEC_GET(MDM_USER_NODE, modem_power_gpios);
static const struct gpio_dt_spec mdm_rst =
	GPIO_DT_SPEC_GET(MDM_USER_NODE, modem_reset_gpios);

/* ---- UART RX path ---------------------------------------------------- */

#define UART_RX_BUF_SIZE 256
#define UART_RX_BUF_NUM  2
static uint8_t uart_rx_bufs[UART_RX_BUF_NUM][UART_RX_BUF_SIZE];
static volatile int uart_rx_active = 0;

RING_BUF_DECLARE(rx_ringbuf, 1024);
static K_SEM_DEFINE(rx_data_avail, 0, 1);

/* TX completion semaphore. Zephyr's async uart_tx returns -EBUSY if a
 * previous transmission is still running, so chained at_send_raw calls
 * would silently drop bytes unless we serialize on UART_TX_DONE. Init
 * at 1 (signaled) so the first send can proceed immediately. */
static K_SEM_DEFINE(tx_done, 1, 1);

/* Lines parsed out of the byte stream are dispatched here. The state
 * machine peeks at the latest message; raw AT passthrough echoes everything
 * to the console for debugging. */
#define MDM_LINE_MAX 256
K_MSGQ_DEFINE(line_msgq, MDM_LINE_MAX, 16, 4);

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);
	switch (evt->type) {
	case UART_TX_DONE:
	case UART_TX_ABORTED:
		k_sem_give(&tx_done);
		break;
	case UART_RX_RDY:
		ring_buf_put(&rx_ringbuf,
			evt->data.rx.buf + evt->data.rx.offset,
			evt->data.rx.len);
		k_sem_give(&rx_data_avail);
		break;
	case UART_RX_BUF_REQUEST: {
		/* hand the other ping-pong buffer back to the driver */
		static int next = 1;
		uart_rx_buf_rsp(dev, uart_rx_bufs[next], UART_RX_BUF_SIZE);
		next = (next + 1) % UART_RX_BUF_NUM;
		break;
	}
	case UART_RX_BUF_RELEASED:
		break;
	case UART_RX_DISABLED:
		uart_rx_active = 0;
		break;
	default:
		break;
	}
}

static int uart_start_rx(void)
{
	if (uart_rx_active) return 0;
	int err = uart_callback_set(uart_dev, uart_cb, NULL);
	if (err) {
		LOG_ERR("uart_callback_set: %d", err);
		return err;
	}
	err = uart_rx_enable(uart_dev, uart_rx_bufs[0], UART_RX_BUF_SIZE,
		/* idle timeout in us */ 5000);
	if (err) {
		LOG_ERR("uart_rx_enable: %d", err);
		return err;
	}
	uart_rx_active = 1;
	return 0;
}

/* MCU<->modem UART baud. Target = high-speed steady state; fallback = BG770A
 * factory default. Bootstrap (try_baud_bootstrap, below near at_probe_alive)
 * auto-detects + persists 921600 with &W on first contact. */
#define MDM_TARGET_BAUD   921600
#define MDM_FALLBACK_BAUD 115200
static int current_uart_baud = MDM_TARGET_BAUD;  /* matches DT current-speed */

/* Switch the UART to a new baud. Caller is responsible for the modem's side
 * being at (or about to switch to) the same rate — otherwise the link is dead
 * until both ends realign. Stops RX, reconfigures, flushes stale bytes, restarts. */
static int set_uart_baud(int baud)
{
	if (baud == current_uart_baud) return 0;

	uart_rx_disable(uart_dev);
	/* uart_rx_disable is async; wait for the UART_RX_DISABLED callback to
	 * flip uart_rx_active before reconfiguring. */
	int64_t deadline = k_uptime_get() + 200;
	while (uart_rx_active && k_uptime_get() < deadline) k_msleep(5);

	struct uart_config c = {
		.baudrate  = baud,
		.parity    = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS,
	};
	int err = uart_configure(uart_dev, &c);
	if (err) {
		LOG_ERR("modem: uart_configure %d -> %d", baud, err);
		(void)uart_start_rx();   /* try to recover RX */
		return err;
	}
	current_uart_baud = baud;

	/* Drop anything that crossed the baud change — it's garbage now. */
	ring_buf_reset(&rx_ringbuf);
	k_msgq_purge(&line_msgq);

	return uart_start_rx();
}

/* Pull bytes out of the ring buffer and split into '\n'-terminated lines.
 * '\r' is dropped. Empty lines are skipped (BG770A pads responses with
 * "\r\n" both before and after).
 *
 * Special case: '>' (the AT prompt for AT+QMTPUB / AT+QIOPEN data input
 * etc.) is emitted as its own pseudo-line so callers can wait for it via
 * at_wait_prefix(">", ...). AT echo is disabled (ATE0) and BG770A only
 * uses '>' for that prompt, so confusing it with response data isn't a
 * concern. Without this, the prompt sits in the ring buffer until the
 * next '\n' arrives — which it doesn't, since the prompt is "> " with
 * no newline — and the sender stalls every publish. */
/* ---- MQTT control + liveness (subscribe path) ----------------------- */
/* We subscribe to a control topic and use it for two things:
 *   - start/stop: a "start"/"stop" message enables/disables data publishing
 *     (the link stays connected so we can be re-started remotely).
 *   - ping/pong liveness: we publish "ping:<n>" to the same topic and expect
 *     it looped back by the broker. A missing echo means the full path
 *     (not just TCP) is dead — more reliable and faster than the modem's
 *     keepalive, and lighter to tune. */
static const char       *mdm_ctrl_topic     = "rideform/0/ctrl";
/* Default OFF: bring the modem + MQTT link up at boot and keep it warm, but
 * don't stream tracker data (which costs cellular data continuously) until the
 * Deck sends "start". The Deck shows a Start button until then. */
static volatile bool     mdm_publish_enabled = false;
static volatile bool     mdm_confirm_pending = false; /* send started/stopped */
static volatile int64_t  mdm_last_pong_ms    = 0;   /* last ping echo seen */

/* Parse an inline +QMTRECV URC and act on it. With recv/mode inline the URC is
 *   +QMTRECV: <idx>,<mid>,"<topic>","<payload>"
 * so the payload is the last double-quoted field. Run from rx_line_thread so
 * control/ping messages are handled regardless of what the state machine is
 * currently waiting on (at_wait_prefix would otherwise discard them). */
static void handle_qmtrecv(const char *line)
{
	const char *close = strrchr(line, '"');
	if (!close) return;
	const char *open = close - 1;
	while (open > line && *open != '"') open--;
	if (*open != '"' || open >= close) return;
	char payload[48];
	size_t n = (size_t)(close - open - 1);
	if (n >= sizeof(payload)) n = sizeof(payload) - 1;
	memcpy(payload, open + 1, n);
	payload[n] = 0;

	/* Any incoming MQTT message — our own ping loopback, the Deck's start/stop,
	 * even garbled bytes from a desynced batch — proves the
	 * broker -> modem -> nRF path is alive right now, so refresh the liveness
	 * timer unconditionally. Otherwise the occasional LTE-dropped ping echo
	 * piles up to 3 misses and trips a false reconnect even when the link is
	 * fine and other traffic is flowing. */
	mdm_last_pong_ms = k_uptime_get();

	if (strncmp(payload, "ping:", 5) == 0) {
		/* already accounted for above */
	} else if (strcmp(payload, "start") == 0) {
		mdm_publish_enabled = true;
		mdm_confirm_pending = true;
		LOG_INF("modem: MQTT cmd -> start");
	} else if (strcmp(payload, "stop") == 0) {
		mdm_publish_enabled = false;
		mdm_confirm_pending = true;
		LOG_INF("modem: MQTT cmd -> stop");
	}
}

static void rx_line_thread(void)
{
	static char buf[MDM_LINE_MAX];
	static int pos = 0;
	while (1) {
		k_sem_take(&rx_data_avail, K_FOREVER);
		uint8_t b;
		while (ring_buf_get(&rx_ringbuf, &b, 1) == 1) {
			if (b == '\r') continue;
			if (b == '\n') {
				if (pos > 0) {
					buf[pos] = 0;
					if (strncmp(buf, "+QMTRECV", 8) == 0) {
						/* Control/ping URC — handle here, don't
						 * enqueue (the state machine would discard it). */
						LOG_INF("RX URC: %s", buf);  /* diag: see actual URC format */
						handle_qmtrecv(buf);
					} else {
						char tmp[MDM_LINE_MAX];
						memcpy(tmp, buf, pos + 1);
						(void)k_msgq_put(&line_msgq, tmp, K_NO_WAIT);
					}
					pos = 0;
				}
				continue;
			}
			if (b == '>') {
				/* Emit ">" as its own line so the data-input
				 * prompt can be awaited without depending on
				 * a following '\n' that never comes. */
				if (pos > 0) {
					buf[pos] = 0;
					char tmp[MDM_LINE_MAX];
					memcpy(tmp, buf, pos + 1);
					(void)k_msgq_put(&line_msgq, tmp, K_NO_WAIT);
					pos = 0;
				}
				char prompt[2] = { '>', 0 };
				(void)k_msgq_put(&line_msgq, prompt, K_NO_WAIT);
				continue;
			}
			if (pos < MDM_LINE_MAX - 1) buf[pos++] = b;
			else pos = 0; /* overflow: drop */
		}
	}
}
K_THREAD_DEFINE(modem_rx_tid, 1024, rx_line_thread, NULL, NULL, NULL, 6, 0, 0);

/* ---- AT TX ----------------------------------------------------------- */

static int at_send_raw(const uint8_t *data, size_t len)
{
	/* Wait for the previous TX to finish — async uart_tx returns
	 * -EBUSY otherwise and the bytes silently disappear. */
	if (k_sem_take(&tx_done, K_MSEC(500)) != 0) {
		LOG_WRN("modem: tx_done timeout");
		return -EBUSY;
	}
	int e = uart_tx(uart_dev, data, len, SYS_FOREVER_US);
	if (e != 0) {
		/* uart_tx never started — give the semaphore back so we
		 * don't deadlock on the next call. */
		k_sem_give(&tx_done);
		LOG_WRN("modem: uart_tx %d", e);
	}
	return e;
}

static int at_send_line(const char *cmd)
{
	char buf[MDM_LINE_MAX];
	int n = snprintf(buf, sizeof(buf), "%s\r", cmd);
	if (n <= 0 || n >= (int)sizeof(buf)) return -EINVAL;
	LOG_DBG("> %s", cmd);
	return at_send_raw((const uint8_t *)buf, n);
}

/* Drain the line queue into a buffer, waiting up to `timeout_ms` for `prefix`.
 * Returns 0 on match (line content copied into out, sized cap), -ETIMEDOUT
 * otherwise. Other lines are logged and discarded. */
static int at_wait_prefix(const char *prefix, char *out, size_t cap, int timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	char line[MDM_LINE_MAX];
	while (1) {
		int64_t now = k_uptime_get();
		k_timeout_t to = (deadline > now)
			? K_MSEC(deadline - now) : K_NO_WAIT;
		int err = k_msgq_get(&line_msgq, line, to);
		if (err) return -ETIMEDOUT;
		LOG_DBG("< %s", line);
		if (strncmp(line, prefix, strlen(prefix)) == 0) {
			if (out && cap) {
				strncpy(out, line, cap - 1);
				out[cap - 1] = 0;
			}
			return 0;
		}
		/* fall through: keep draining */
	}
}

/* ---- Settings (NVS-backed) ------------------------------------------- */

typedef struct {
	char apn[MDM_APN_MAX];
	char host[MDM_HOST_MAX];
	uint16_t port;
	char client[MDM_CLIENT_MAX];
	char topic[MDM_TOPIC_MAX];
	char user[MDM_USER_MAX];
	char pass[MDM_PASS_MAX];
} mdm_cfg_t;

static mdm_cfg_t cfg;

static void cfg_load(void)
{
	memset(&cfg, 0, sizeof(cfg));
	sys_read(MDM_NVS_APN,         cfg.apn,    sizeof(cfg.apn));
	sys_read(MDM_NVS_MQTT_HOST,   cfg.host,   sizeof(cfg.host));
	sys_read(MDM_NVS_MQTT_PORT,   &cfg.port,  sizeof(cfg.port));
	sys_read(MDM_NVS_MQTT_CLIENT, cfg.client, sizeof(cfg.client));
	sys_read(MDM_NVS_MQTT_TOPIC,  cfg.topic,  sizeof(cfg.topic));
	sys_read(MDM_NVS_MQTT_USER,   cfg.user,   sizeof(cfg.user));
	sys_read(MDM_NVS_MQTT_PASS,   cfg.pass,   sizeof(cfg.pass));
	if (cfg.port == 0) cfg.port = 1883;
}

/* ---- Tracker packet ring (batched publish) -------------------------- */
/* A single chronological SPSC ring of raw 16-byte ESB packets. The ESB
 * event_handler (ISR context) is the only producer; the modem thread is the
 * only consumer, so head/tail need no lock — aligned 16-bit accesses are
 * atomic on Cortex-M and each side only advances its own index.
 *
 * Why a FIFO of every packet rather than the old "latest per id" snapshot:
 * the LTE-M / BG770A AT path tops out at ~10 QMTPUB/s, so we publish slowly
 * (~5 Hz) but pack every sample received in the interval into one message. At
 * a 30 Hz per-tracker cap that's ~6 samples/tracker/publish — the Deck takes
 * the last for live (and can replay the rest later), recovering ~30 Hz of
 * temporal resolution over a link that can only carry ~5 messages/s.
 * Chronological order is preserved, which is exactly what time-replay wants.
 *
 * MDM_TXRING_SZ must be a power of two (mask wrap). 256 packets = 4 KB,
 * ~0.85 s of buffer at 300 pkt/s (10 trackers × 30 Hz). */
/* 20 bytes forwarded per ESB packet: the 16B SlimeVR payload + the tracker's
 * 4B CRC32 (rx_payload.data[16..19], the seq byte [20] is dropped). The
 * receiver already verified this CRC over the air (esb.c), but the nRF->BG770A
 * UART leg has no checksum, so we carry the CRC through to the Deck for an
 * end-to-end integrity check that pinpoints UART/MQTT-path corruption. */
#define MDM_PKT_BYTES    20
#define MDM_TXRING_SZ    256
#define MDM_TXRING_MASK  (MDM_TXRING_SZ - 1)

static uint8_t           txring[MDM_TXRING_SZ][MDM_PKT_BYTES];
static volatile uint16_t txr_head;    /* producer (ISR) advances */
static volatile uint16_t txr_tail;    /* consumer (modem thread) advances */
static volatile uint32_t txr_dropped; /* ring-full drops, diagnostics */

bool modem_enqueue_tracker_packet(const uint8_t *pkt16)
{
	uint16_t head = txr_head;
	uint16_t next = (head + 1) & MDM_TXRING_MASK;
	if (next == txr_tail) {
		/* Ring full — consumer stalled (modem wedged/reconnecting). Drop
		 * the newest sample rather than break the SPSC invariant by
		 * touching tail from the producer side. */
		txr_dropped++;
		return false;
	}
	memcpy(txring[head], pkt16, MDM_PKT_BYTES);
	txr_head = next;   /* publish the slot only after it is fully written */
	return true;
}

/* ---- State machine --------------------------------------------------- */

typedef enum {
	MDM_OFF = 0,
	MDM_BOOT_PWR,
	MDM_AT_PROBE,
	MDM_AT_CONFIG,
	MDM_SIM_WAIT,
	MDM_NET_REG,
	MDM_PDP_ACT,
	MDM_MQTT_OPEN,
	MDM_MQTT_CONN,
	MDM_READY,
	MDM_FAULT,
} mdm_state_t;

static const char *state_names[] = {
	"OFF", "BOOT_PWR", "AT_PROBE", "AT_CONFIG", "SIM_WAIT", "NET_REG",
	"PDP_ACT", "MQTT_OPEN", "MQTT_CONN", "READY", "FAULT",
};

static volatile mdm_state_t state = MDM_OFF;
static volatile bool start_requested = false;

const char *modem_state_string(void)
{
	int i = (int)state;
	if (i < 0 || i >= (int)ARRAY_SIZE(state_names)) return "?";
	return state_names[i];
}

void modem_request_start(void) { start_requested = true; }
void modem_request_stop(void)  { start_requested = false; state = MDM_OFF; }

/* Each step returns next state; placeholders log the intent so we can wire
 * up real AT exchanges once we have a board on the bench. */
static mdm_state_t step_off(void)
{
	if (!start_requested) { k_msleep(200); return MDM_OFF; }
	LOG_INF("modem: start requested -> BOOT_PWR");
	return MDM_BOOT_PWR;
}

static int consecutive_faults = 0;

/* Called when AT was silent at the current (target) baud. If the modem turns
 * out to be at factory 115200 (fresh module or NVM reset), set IPR to
 * MDM_TARGET_BAUD with &W (persist), switch our UART up, and continue.
 * Cost when modem is genuinely silent: ~5s extra per probe failure. */
static bool try_baud_bootstrap(void)
{
	if (current_uart_baud != MDM_TARGET_BAUD) return false;

	LOG_INF("modem: AT silent at %d — trying %d (factory)",
	        MDM_TARGET_BAUD, MDM_FALLBACK_BAUD);
	if (set_uart_baud(MDM_FALLBACK_BAUD) != 0) return false;

	bool alive = false;
	int64_t deadline = k_uptime_get() + 5000;
	do {
		at_send_line("AT");
		if (at_wait_prefix("OK", NULL, 0, 800) == 0) { alive = true; break; }
	} while (k_uptime_get() < deadline);

	if (!alive) {
		/* Modem really is off/wedged, not just baud-mismatched. Restore
		 * target baud so normal escalation (PWRKEY/RESET) takes over. */
		(void)set_uart_baud(MDM_TARGET_BAUD);
		return false;
	}

	LOG_INF("modem: alive at %d — persisting IPR=%d;&W",
	        MDM_FALLBACK_BAUD, MDM_TARGET_BAUD);
	/* OK reply may come back at the NEW baud (Quectel switches on the stop
	 * bit of the OK), so don't try to read it — push the command, give it a
	 * moment, then switch and verify with a fresh probe. */
	at_send_line("AT+IPR=921600;&W");
	k_msleep(300);

	if (set_uart_baud(MDM_TARGET_BAUD) != 0) return false;

	deadline = k_uptime_get() + 2000;
	do {
		at_send_line("AT");
		if (at_wait_prefix("OK", NULL, 0, 800) == 0) {
			LOG_INF("modem: baud bootstrap OK — running at %d", MDM_TARGET_BAUD);
			return true;
		}
	} while (k_uptime_get() < deadline);

	LOG_WRN("modem: baud bootstrap: IPR set but no AT at %d", MDM_TARGET_BAUD);
	return false;
}

/* Send AT every second for up to timeout_ms, return true on first "OK".
 * Used to detect whether the BG770A is alive *without* touching PWRKEY.
 *
 * Falls back to a baud bootstrap if the timeout expires: a factory-fresh
 * BG770A is at 115200, which won't answer when we're at 921600. */
static bool at_probe_alive(int timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	do {
		at_send_line("AT");
		if (at_wait_prefix("OK", NULL, 0, 1000) == 0) return true;
	} while (k_uptime_get() < deadline);
	return try_baud_bootstrap();
}

/* BG770A's PWRKEY is a toggle (pulse-while-OFF powers on, pulse-while-ON
 * shuts down) and this board does not wire the module's STATUS line back
 * to the nRF52, so we cannot read the power state directly. Blindly
 * pulsing on every start used to shut an already-on module down, costing
 * ~7 min of FAULT recovery to climb back out.
 *
 * Robust approach: probe AT first. If the module answers — already on, or
 * still auto-booting after VBAT — skip the pulse entirely. Pulse only when
 * the module is genuinely silent. This is idempotent across FAULT retries
 * too: a transient downstream fault re-enters here, re-probes (fast when
 * the module is up), and never toggles a live module off. */
static mdm_state_t step_boot_pwr(void)
{
	/* 16s window covers the ~13s Quectel cold-boot-to-RDY, so an
	 * auto-powered module is caught here rather than mistaken for off. */
	if (at_probe_alive(16000)) {
		LOG_INF("modem: AT alive without pulse — module already on");
		consecutive_faults = 0;
		return MDM_AT_CONFIG;
	}

	/* Escalation. PWRKEY is a TOGGLE on the BG770A but the timing matters:
	 * short pulse (~500ms) = power-ON spec; long pulse (>2s) = power-OFF spec.
	 * A 700ms pulse on a module that's stuck "on but unresponsive" is a no-op,
	 * which is why we used to loop forever. Step up by attempt count:
	 *   1-2 fails: short PWRKEY (normal power-on attempt)
	 *   3-4 fails: LONG PWRKEY (forces OFF) then short pulse to bring back on
	 *   5+  fails: RESET pin — hardware-level reset, ignores modem state */
	if (consecutive_faults <= 1) {
		LOG_INF("modem: no AT response — short PWRKEY (try power-on)");
		gpio_pin_set_dt(&mdm_pwr, 1);
		k_msleep(700);
		gpio_pin_set_dt(&mdm_pwr, 0);
		k_msleep(13000);
	} else if (consecutive_faults <= 3) {
		LOG_WRN("modem: AT silent (fault=%d) — LONG PWRKEY to force OFF",
		        consecutive_faults);
		gpio_pin_set_dt(&mdm_pwr, 1);
		k_msleep(3000);       /* >2s = shutdown spec */
		gpio_pin_set_dt(&mdm_pwr, 0);
		k_msleep(2000);       /* let it settle in OFF */
		LOG_INF("modem: short PWRKEY to power back on");
		gpio_pin_set_dt(&mdm_pwr, 1);
		k_msleep(700);
		gpio_pin_set_dt(&mdm_pwr, 0);
		k_msleep(13000);
	} else {
		LOG_WRN("modem: AT silent (fault=%d) — RESET pin hard reset",
		        consecutive_faults);
		gpio_pin_set_dt(&mdm_rst, 1);
		k_msleep(200);        /* hold reset asserted ~200ms */
		gpio_pin_set_dt(&mdm_rst, 0);
		k_msleep(13000);      /* let it boot back up */
	}
	(void)at_wait_prefix("RDY", NULL, 0, 500);
	return MDM_AT_PROBE;
}

static mdm_state_t step_at_probe(void)
{
	/* 15s (was 5s): this runs right after either a PWRKEY pulse (step_boot_pwr
	 * already waited 13s) or an AT+CFUN=1,1 reboot (step_at_config waited 15s),
	 * but a BG770A can still need a few extra seconds to accept AT after RDY.
	 * A too-short window dropped us into FAULT mid-boot, and the FAULT->BOOT_PWR
	 * retry could then PWRKEY-toggle a module that was merely still booting —
	 * the runaway loop behind "consecutive=10, won't recover". */
	if (at_probe_alive(15000)) {
		consecutive_faults = 0;  /* AT round-trip ok, healthy */
		return MDM_AT_CONFIG;
	}
	LOG_WRN("modem: AT probe timed out");
	return MDM_FAULT;
}

/* One-shot band/mode provisioning. We rewrite QCFG band+iotopmode+nwscanseq
 * and the PDP context, then reboot the module so they take effect. Running
 * this every time AT_CONFIG is re-entered (e.g. via FAULT retry) would loop
 * the modem forever, so we latch it to one shot per nRF52 run. */
static bool band_provisioned = false;

static mdm_state_t step_at_config(void)
{
	/* Gate everything below on a confirmed AT round-trip. A desynced or
	 * still-booting module silently times out every command — yet the old
	 * code would charge ahead, run AT+CFUN=1,1 into the void, and latch
	 * band_provisioned=true against a dead UART (so the one-shot provisioning
	 * was wasted and never retried). Bounce back to AT_PROBE instead; only a
	 * module that actually answered ATE0 proceeds. */
	at_send_line("ATE0");
	if (at_wait_prefix("OK", NULL, 0, 1000) != 0) {
		LOG_WRN("modem: ATE0 got no OK — module not ready, re-probing");
		return MDM_AT_PROBE;
	}
	at_send_line("AT+CMEE=1");
	(void)at_wait_prefix("OK", NULL, 0, 500);

	if (!band_provisioned) {
		/* Found via SeeedJP Wio BG770A wiki + Soracom community: for
		 * Soracom plan-D (NTT Docomo MVNO) on Quectel BG770A-GL, the
		 * default band+iotopmode lets the module wander into NB-IoT
		 * or out-of-region bands and EPS attach returns CEREG stat=3
		 * (registration denied). Locking to LTE-M only with Docomo
		 * band mask makes attach succeed within 30-60s.
		 *
		 * Sequence: RF off -> set QCFG -> set PDP -> CFUN=1,1 (reboot)
		 * -> wait -> re-probe AT. The reboot is mandatory for QCFG
		 * changes to take effect even with effect=1. */
		LOG_INF("modem: provisioning band/iotopmode (one-shot)");

		at_send_line("AT+CFUN=0");
		(void)at_wait_prefix("OK", NULL, 0, 5000);

		/* iotopmode: 0=eMTC only (== LTE-M), 1=NB-IoT, 2=auto. The
		 * trailing 0 = take effect after reboot. */
		at_send_line("AT+QCFG=\"iotopmode\",0,0");
		(void)at_wait_prefix("OK", NULL, 0, 1000);

		/* Scan order: eMTC (02) -> NB-IoT (03) -> GSM (01). BG770A
		 * has no GSM but the trailing 01 is harmless. */
		at_send_line("AT+QCFG=\"nwscanseq\",020301,0");
		(void)at_wait_prefix("OK", NULL, 0, 1000);

		/* Band mask, NTT Docomo: LTE-M = 0xa040005 (B1+B3+B18+B19+B28),
		 * NB-IoT = 0x8040005. First 0 = GSM band (unused on BG770A),
		 * trailing 0 = take effect after reboot. */
		at_send_line("AT+QCFG=\"band\",0,0xa040005,0x8040005,0");
		(void)at_wait_prefix("OK", NULL, 0, 1000);

		/* PDP context IPv4 only (IPv4v6 sometimes denied by Docomo MVNOs). */
		char buf[160];
		snprintf(buf, sizeof(buf),
		         "AT+CGDCONT=1,\"IP\",\"%s\"",
		         cfg.apn[0] ? cfg.apn : "soracom.io");
		at_send_line(buf);
		(void)at_wait_prefix("OK", NULL, 0, 1000);

		/* QICSGP credentials: Soracom requires sora/sora over CHAP (2). */
		snprintf(buf, sizeof(buf),
		         "AT+QICSGP=1,1,\"%s\",\"sora\",\"sora\",2",
		         cfg.apn[0] ? cfg.apn : "soracom.io");
		at_send_line(buf);
		(void)at_wait_prefix("OK", NULL, 0, 1000);

		LOG_INF("modem: AT+CFUN=1,1 (module reboot, ~13s)");
		at_send_line("AT+CFUN=1,1");
		k_msleep(15000);  /* BG770A cold-boot time */

		band_provisioned = true;
		/* Bounce back to AT_PROBE so we confirm the module is alive
		 * after the reboot before we walk into SIM_WAIT. */
		return MDM_AT_PROBE;
	}

	return MDM_SIM_WAIT;
}

static mdm_state_t step_sim_wait(void)
{
	/* TODO(hardware): poll AT+CPIN? until READY, with a sane retry budget. */
	at_send_line("AT+CPIN?");
	char line[MDM_LINE_MAX];
	if (at_wait_prefix("+CPIN: READY", line, sizeof(line), 5000) == 0)
		return MDM_NET_REG;
	LOG_WRN("modem: SIM not ready");
	k_msleep(1000);
	return MDM_SIM_WAIT;
}

static mdm_state_t step_net_reg(void)
{
	/* Poll CSQ + CEREG and log both — this is the most common stall
	 * spot so we want maximum visibility on the console.
	 *
	 * CSQ:  +CSQ: <rssi>,<ber>   rssi 0..31 (99 = unknown). >10 is usable.
	 * CEREG:+CEREG: <n>,<stat>[,...]  stat: 0=not reg/not searching,
	 *                                       1=registered home,
	 *                                       2=searching,
	 *                                       3=registration denied,
	 *                                       4=unknown,
	 *                                       5=registered roaming. */
	char line[MDM_LINE_MAX];

	at_send_line("AT+CSQ");
	if (at_wait_prefix("+CSQ:", line, sizeof(line), 2000) == 0) {
		LOG_INF("modem: %s", line);
	}

	at_send_line("AT+CEREG?");
	if (at_wait_prefix("+CEREG:", line, sizeof(line), 5000) == 0) {
		LOG_INF("modem: %s", line);
		int n = 0, stat = 0;
		if (sscanf(line, "+CEREG: %d,%d", &n, &stat) >= 2) {
			if (stat == 1 || stat == 5) {
				LOG_INF("modem: registered (stat=%d)", stat);
				return MDM_PDP_ACT;
			}
		}
	}
	k_msleep(3000);
	return MDM_NET_REG;
}

static mdm_state_t step_pdp_act(void)
{
	/* QICSGP was already programmed (sora/sora/CHAP) in step_at_config —
	 * rewriting it here with empty user/pass+PAP would clobber that and
	 * QIACT would silently fail. So this step only activates the context.
	 *
	 * Soracom plan-D activation can take up to ~60s on first attach. */
	at_send_line("AT+QIACT=1");
	if (at_wait_prefix("OK", NULL, 0, 60000) == 0) {
		/* Read back the assigned IP so we know the context is really
		 * up — an OK without an IP would be useless (and QMTOPEN
		 * would silently time out). */
		at_send_line("AT+QIACT?");
		char line[MDM_LINE_MAX];
		if (at_wait_prefix("+QIACT:", line, sizeof(line), 5000) == 0) {
			LOG_INF("modem: %s", line);
		} else {
			LOG_WRN("modem: +QIACT? gave no response (context not really up?)");
		}
		return MDM_MQTT_OPEN;
	}

	/* Capture the underlying error code for the log so we can tell
	 * "auth wrong" from "timeout" from "context busy" etc. */
	char err[MDM_LINE_MAX];
	at_send_line("AT+QIGETERROR");
	if (at_wait_prefix("+QIGETERROR:", err, sizeof(err), 2000) == 0) {
		LOG_WRN("modem: %s", err);
	}
	LOG_WRN("modem: PDP activation failed");
	return MDM_FAULT;
}

/* MQTT-layer failure streak. Separate from consecutive_faults (which the
 * AT probe in BOOT_PWR resets whenever the module answers AT) because a
 * wedged MQTT stack still passes AT — so we'd loop FAULT->BOOT_PWR->
 * MQTT_OPEN->fail forever. This counter survives the AT-probe reset. */
static int mqtt_fail_streak = 0;
#define MDM_MQTT_FAIL_HARD_RESET      4   /* radio/PDP looks dead — reboot soon */
#define MDM_MQTT_FAIL_HARD_RESET_PDP 30   /* PDP alive (broker likely down) — rarely */

/* True when step_mqtt_open last saw an active PDP context (AT+QIACT?). If the
 * data link is up but MQTT won't connect, the broker is probably down/メンテ —
 * rebooting the radio is pointless and just burns data + battery on LTE
 * re-attaches. So only escalate to a full CFUN reset quickly when the radio
 * itself looks dead; when the PDP is healthy, keep soft-retrying QMTOPEN (which
 * succeeds the moment the broker returns) and reset the radio only as a very
 * last resort. */
static volatile bool mdm_pdp_alive = false;

/* Escalating MQTT failure handler. Soft retry (FAULT -> backoff -> re-probe ->
 * QMTCLOSE/QMTOPEN) for most failures; AT+CFUN=1,1 full radio reboot only when
 * the streak is long enough to suspect a wedged stack — and far more patiently
 * when the PDP context is up (broker-down case). */
static mdm_state_t mqtt_fail(void)
{
	int threshold = mdm_pdp_alive ? MDM_MQTT_FAIL_HARD_RESET_PDP
	                              : MDM_MQTT_FAIL_HARD_RESET;
	if (++mqtt_fail_streak >= threshold) {
		LOG_WRN("modem: %d MQTT failures (pdp_alive=%d) — AT+CFUN=1,1 modem reset",
		        mqtt_fail_streak, (int)mdm_pdp_alive);
		mqtt_fail_streak = 0;
		at_send_line("AT+CFUN=1,1");
		k_msleep(2000);   /* module starts rebooting (~13s to RDY) */
		return MDM_BOOT_PWR;  /* re-probe AT, then full chain re-init */
	}
	return MDM_FAULT;
}

static mdm_state_t step_mqtt_open(void)
{
	if (!cfg.host[0]) {
		LOG_WRN("modem: MQTT host not configured");
		k_msleep(2000);
		return MDM_MQTT_OPEN;
	}

	char buf[160];
	char line[MDM_LINE_MAX];

	/* Tear down any stale session first. On a reconnect (publish-failure
	 * recovery) socket 0 may still be half-open on the module; QMTOPEN
	 * would then return "already in use". Disconnect + close are no-ops
	 * on a fresh boot, so we ignore their results either way. */
	at_send_line("AT+QMTDISC=0");
	(void)at_wait_prefix("+QMTDISC:", line, sizeof(line), 2000);
	at_send_line("AT+QMTCLOSE=0");
	(void)at_wait_prefix("+QMTCLOSE:", line, sizeof(line), 2000);

	/* First confirm the PDP context is really up by re-asking — if QIACT
	 * stuck without an IP, everything below silently fails. */
	at_send_line("AT+QIACT?");
	if (at_wait_prefix("+QIACT:", line, sizeof(line), 3000) == 0) {
		LOG_INF("modem: %s", line);
		mdm_pdp_alive = true;   /* data link up — a QMTOPEN fail = broker, not radio */
	} else {
		LOG_WRN("modem: no PDP context active");
		mdm_pdp_alive = false;
	}

	/* Try a ping to the broker host — this exercises DNS resolution AND
	 * the TCP/IP path. +QPING reports per-packet timing and a final
	 * summary, which makes "DNS fails" vs "ICMP blocked" vs "host
	 * unreachable" easy to read off the log. */
	snprintf(buf, sizeof(buf), "AT+QPING=1,\"%s\",4,2", cfg.host);
	at_send_line(buf);
	for (int i = 0; i < 8; i++) {
		if (at_wait_prefix("+QPING:", line, sizeof(line), 4000) != 0) break;
		LOG_INF("modem: %s", line);
		/* Final summary line is "+QPING: <finresult>,<sent>,<rcvd>,..." */
		int v = -1; if (sscanf(line, "+QPING: %d,%*d,%*d", &v) == 1) break;
	}

	/* Explicit DNS resolve too, since QMTOPEN-by-hostname has historically
	 * been finicky on Quectel modems with some carriers. */
	snprintf(buf, sizeof(buf), "AT+QIDNSGIP=1,\"%s\"", cfg.host);
	at_send_line(buf);
	for (int i = 0; i < 3; i++) {
		if (at_wait_prefix("+QIURC:", line, sizeof(line), 8000) != 0) break;
		LOG_INF("modem: %s", line);
		if (strstr(line, "\"dnsgip\",\"")) break;  /* got the IP line */
	}

	snprintf(buf, sizeof(buf),
		"AT+QMTOPEN=0,\"%s\",%u", cfg.host, cfg.port);
	at_send_line(buf);

	/* +QMTOPEN: <idx>,<result>
	 *   result: 0=ok, 1=param, 2=busy, 3=PDP not active, 4=parse, 5=id. */
	if (at_wait_prefix("+QMTOPEN:", line, sizeof(line), 30000) == 0) {
		LOG_INF("modem: %s", line);
		int idx = 0, rc = -1;
		if (sscanf(line, "+QMTOPEN: %d,%d", &idx, &rc) == 2 && rc == 0) {
			return MDM_MQTT_CONN;
		}
	}
	LOG_WRN("modem: QMTOPEN failed (host=%s port=%u)", cfg.host, cfg.port);
	return mqtt_fail();
}

static mdm_state_t step_mqtt_conn(void)
{
	char buf[160];
	/* Keepalive as a backstop (the app-level ping/pong below is the primary,
	 * faster liveness check). Must be set before QMTCONN. */
	at_send_line("AT+QMTCFG=\"keepalive\",0,20");
	(void)at_wait_prefix("OK", NULL, 0, 1000);
	/* Inline receive: the payload arrives directly in the +QMTRECV URC, so
	 * rx_line_thread can act on control/ping messages with no read step. */
	at_send_line("AT+QMTCFG=\"recv/mode\",0,0,1");
	(void)at_wait_prefix("OK", NULL, 0, 1000);

	if (cfg.user[0]) {
		snprintf(buf, sizeof(buf),
			"AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
			cfg.client[0] ? cfg.client : "rideform",
			cfg.user, cfg.pass);
	} else {
		snprintf(buf, sizeof(buf),
			"AT+QMTCONN=0,\"%s\"",
			cfg.client[0] ? cfg.client : "rideform");
	}
	at_send_line(buf);
	if (at_wait_prefix("+QMTCONN: 0,0,0", NULL, 0, 10000) == 0) {
		LOG_INF("modem: connected to %s:%u", cfg.host, cfg.port);
		mqtt_fail_streak = 0;   /* healthy session — clear escalation */
		/* Subscribe to the control topic: start/stop commands + the loopback
		 * of our own liveness pings. */
		snprintf(buf, sizeof(buf), "AT+QMTSUB=0,1,\"%s\",0", mdm_ctrl_topic);
		at_send_line(buf);
		(void)at_wait_prefix("+QMTSUB:", NULL, 0, 5000);
		mdm_last_pong_ms = k_uptime_get();  /* seed so liveness doesn't instant-trip */
		return MDM_READY;
	}
	LOG_WRN("modem: QMTCONN failed");
	return mqtt_fail();
}

/* Drain the snapshot and publish via QMTPUB binary mode.
 *
 * Critical-path timing — this is called from step_ready ~30Hz. Every ms
 * here delays the next publish, so all the waits are intentionally short
 * and any failure just drops the current frame and lets the next one go. */
/* Tunable publish knobs (RAM only — for live ceiling-probing via console;
 * power cycle reverts to these defaults).
 *   pace: min ms between publish attempts. Lower = push harder.
 *   ptmo: how long to wait for the ">" prompt before declaring a miss. */
/* Snapshot publish (latest-per-tracker, small ~160B payloads) at pace 10ms
 * gives a steady ~20-40Hz. ptmo 200 is generous on purpose: a too-short
 * timeout that fires while the modem is mid-prompt skips the payload send
 * but leaves the modem waiting for it, desyncing the byte stream (the next
 * command gets read as binary payload -> corrupt publishes). A real miss
 * now only means a dead session, which the recovery path handles.
 * (An earlier "lossless batch" attempt corrupted on large payloads, but that
 * was the prompt-miss desync — fixed now, so batching is back, see the ring
 * above and the drain in publish_frame.) */
/* Publish cadence. With batching (every buffered sample packed per message)
 * the goal is to stay well under the BG770A's ~10 QMTPUB/s AT-command ceiling,
 * NOT to publish fast: ~5 Hz, each message carrying every sample received
 * since the last. pace 150ms lands near that once per-publish UART TX + prompt
 * time is added on top. Live-tunable via `modem pace <ms>`. */
static int mdm_pub_pace_ms   = 150;
static int mdm_prompt_tmo_ms = 200;

void modem_set_pace(int ms)  { mdm_pub_pace_ms   = (ms < 0)  ? 0  : ms; }
void modem_set_ptmo(int ms)  { mdm_prompt_tmo_ms = (ms < 10) ? 10 : ms; }
int  modem_get_pace(void)    { return mdm_pub_pace_ms; }
int  modem_get_ptmo(void)    { return mdm_prompt_tmo_ms; }

/* Returns: 1 = published, 0 = nothing to send, -1 = publish failed
 * (no ">" prompt — modem backlogged or MQTT session gone). step_ready
 * uses the -1 count to decide when to force a reconnect. */
/* Max packets drained into one QMTPUB. Bounds the payload (128×16 = 2 KB, well
 * within the BG770A's QMTPUB limit and ~178 ms of UART TX) and caps how much a
 * backlog can balloon a single message. */
#define MDM_BATCH_PUB_MAX 128

static int publish_frame(void)
{
	/* Snapshot the ring extent. The producer may keep appending past `head`
	 * during the drain; those samples land in the next frame. */
	uint16_t head = txr_head;
	uint16_t tail = txr_tail;
	uint16_t avail = (head - tail) & MDM_TXRING_MASK;
	if (avail == 0) return 0;

	if (avail > MDM_BATCH_PUB_MAX) {
		/* Backlog: drop the oldest overflow so the message stays bounded
		 * and what we send is the most recent. */
		tail = (tail + (avail - MDM_BATCH_PUB_MAX)) & MDM_TXRING_MASK;
		avail = MDM_BATCH_PUB_MAX;
	}

	/* static, not on the stack: only the modem thread calls publish_frame,
	 * and 2 KB would blow the 2 KB thread stack. */
	static uint8_t out[MDM_BATCH_PUB_MAX * MDM_PKT_BYTES];
	for (uint16_t i = 0; i < avail; i++)
		memcpy(out + i * MDM_PKT_BYTES,
		       txring[(tail + i) & MDM_TXRING_MASK], MDM_PKT_BYTES);
	txr_tail = (tail + avail) & MDM_TXRING_MASK;   /* consume */

	int n = avail;
	int payload_len = n * MDM_PKT_BYTES;

	const char *topic = cfg.topic[0] ? cfg.topic : "rideform/0/frame";
	char hdr[160];
	int hl = snprintf(hdr, sizeof(hdr),
		"AT+QMTPUB=0,0,0,0,\"%s\",%d\r", topic, payload_len);
	if (hl <= 0) return 0;
	at_send_raw((const uint8_t *)hdr, hl);

	/* Diagnostic counters — rate-limited to one log per second so they
	 * don't drown the console at 30 Hz. */
	static int64_t s_last_log_ms = 0;
	static int s_pub_ok = 0, s_pub_prompt_miss = 0;
	static int s_pkts_sent = 0;
	static int s_consec_miss = 0;
	int result = 1;

	/* Wait for the ">" prompt — emitted by BG770A right before it expects
	 * payload bytes. rx_line_thread emits ">" as its own line so we can
	 * just wait_prefix on it. Timeout is short (150ms): when the modem is
	 * keeping up the prompt lands in a few ms, and a miss shouldn't stall
	 * the whole loop — a long timeout here is what makes the publish rate
	 * jitter wildly under backlog. */
	if (at_wait_prefix(">", NULL, 0, mdm_prompt_tmo_ms) != 0) {
		s_pub_prompt_miss++;
		/* Drain whatever did arrive. An explicit "ERROR"/"+CME ERROR" means
		 * QMTPUB was rejected outright — no data-input mode opened, so the
		 * MQTT session is gone. */
		char line[MDM_LINE_MAX];
		bool saw_error = false;
		while (k_msgq_get(&line_msgq, line, K_NO_WAIT) == 0) {
			LOG_INF("modem: pub-miss rsp: %s", line);
			if (strstr(line, "ERROR")) saw_error = true;
		}
		if (saw_error || ++s_consec_miss >= 30) {
			/* Real error, or the prompt has gone missing for ~30 frames
			 * straight (link wedged). Force a reconnect — the QMTCLOSE/
			 * QMTOPEN there also clears any half-open data-input mode. */
			result = -1;
			s_consec_miss = 0;
			goto maybe_log;
		}
		/* Otherwise the prompt was merely late: under ESB load the modem
		 * thread gets starved and reads "> " a beat after it arrives. The
		 * module HAS entered data mode and is counting down payload_len
		 * bytes, so we MUST still send the payload — skipping it (the old
		 * behaviour) left the module wedged waiting for bytes that never
		 * came, desyncing every subsequent publish until a reconnect.
		 * Fall through and send, just late. */
	} else {
		s_consec_miss = 0;
	}

	/* Single contiguous send — the TX serialisation semaphore in
	 * at_send_raw means the whole block clears before we move on. */
	at_send_raw(out, payload_len);

	/* Optimization #1: do NOT block waiting for the "+QMTPUB:" URC.
	 * It's QoS 0, so we never retry per-message — the ack carries no
	 * actionable info and the wait was the dominant per-publish cost
	 * (~500ms worst case, capping us near ~17Hz). The URC still arrives
	 * asynchronously; the next publish's at_wait_prefix(">") drains it
	 * (it discards non-matching lines). line_msgq depth (16) easily
	 * absorbs the 1 stale ack between publishes.
	 *
	 * ack_miss is retired; s_pub_ok now counts "frames pushed". */
	s_pub_ok++;
	s_pkts_sent += n;

maybe_log:
	{
		int64_t now = k_uptime_get();
		if (now - s_last_log_ms > 1000) {
			s_last_log_ms = now;
			LOG_INF("modem: pub ok=%d prompt_miss=%d pkts=%d",
			        s_pub_ok, s_pub_prompt_miss, s_pkts_sent);
			s_pub_ok = 0;
			s_pub_prompt_miss = 0;
			s_pkts_sent = 0;
		}
	}
	return result;
}

/* Consecutive publish failures before we assume the MQTT session is dead
 * (vs. transient backlog) and force a full reconnect. At ~30Hz with the
 * default prompt timeout, ~20 misses ≈ a few seconds of nothing through. */
#define MDM_PUB_FAIL_RECONNECT  20

/* Liveness ping/pong: publish a token to the control topic we subscribe to and
 * expect the broker to loop it back (handle_qmtrecv refreshes mdm_last_pong_ms).
 * A QMTPUB can "succeed" locally into a silently dead TCP session (no ERROR, so
 * the publish-failure streak never trips) — this round-trip check sees that
 * because the echo stops arriving. Faster and lighter than the modem keepalive. */
#define MDM_PING_PERIOD_MS       5000   /* while streaming */
#define MDM_PING_PERIOD_IDLE_MS  30000  /* while stopped — just keep the link warm */

/* Publish a short text payload (control / ping). Runs only from the modem
 * thread, so it serialises with the data publishes. */
static void mqtt_publish_text(const char *topic, const char *str)
{
	int sl = (int)strlen(str);
	char hdr[96];
	int hl = snprintf(hdr, sizeof(hdr),
		"AT+QMTPUB=0,0,0,0,\"%s\",%d\r", topic, sl);
	at_send_raw((const uint8_t *)hdr, hl);
	/* Wait briefly for the data-input prompt — but send the payload either way.
	 * Skipping it on a late/missed prompt leaves the modem in data mode waiting
	 * for `sl` bytes; it then consumes the NEXT AT command's bytes as payload
	 * and the publish stream desyncs (we saw the broker receive "AT+QMTP…" as
	 * the body of a ping). publish_frame already does this for data; ping/
	 * start/stop need the same protection. */
	(void)at_wait_prefix(">", NULL, 0, 500);
	at_send_raw((const uint8_t *)str, sl);
}

static mdm_state_t step_ready(void)
{
	static int consecutive_pub_fail = 0;
	static int64_t last_ping_ms = 0;

	/* Acknowledge a start/stop command back to the Deck. Done here (modem
	 * thread) rather than in handle_qmtrecv (rx ISR) so all TX is serialised. */
	if (mdm_confirm_pending) {
		mdm_confirm_pending = false;
		mqtt_publish_text(mdm_ctrl_topic,
		                  mdm_publish_enabled ? "started" : "stopped");
	}

	if (mdm_publish_enabled) {
		int r = publish_frame();
		if (r < 0) {
			if (++consecutive_pub_fail >= MDM_PUB_FAIL_RECONNECT) {
				LOG_WRN("modem: %d publish failures — reconnecting MQTT",
				        consecutive_pub_fail);
				consecutive_pub_fail = 0;
				return MDM_MQTT_OPEN;  /* re-open TCP + MQTT from scratch */
			}
		} else if (r > 0) {
			consecutive_pub_fail = 0;  /* a good publish clears the streak */
		}
	}

	/* Ping/pong liveness. Slower cadence while stopped (the link only needs to
	 * stay warm). The dead-link threshold scales with the period so a longer
	 * idle cadence doesn't trip a false reconnect between pings. */
	int64_t now = k_uptime_get();
	int ping_period = mdm_publish_enabled ? MDM_PING_PERIOD_MS
	                                      : MDM_PING_PERIOD_IDLE_MS;
	if (now - last_ping_ms > ping_period) {
		last_ping_ms = now;
		char ping[24];
		static uint32_t seq = 0;
		snprintf(ping, sizeof(ping), "ping:%u", ++seq);
		mqtt_publish_text(mdm_ctrl_topic, ping);
		if (now - mdm_last_pong_ms > (int64_t)ping_period * 3) {
			LOG_WRN("modem: no MQTT ping echo (>%dms) — reconnecting",
			        ping_period * 3);
			return MDM_MQTT_OPEN;
		}
	}

	k_msleep(mdm_pub_pace_ms);
	return MDM_READY;
}

static mdm_state_t step_fault(void)
{
	/* Distinguish "modem is broken" from "broker is unreachable". When the PDP
	 * data link is up the modem is fine — the issue is upstream (broker down,
	 * home internet flapping, etc.) and could last hours. Counting those
	 * against the modem-escalation budget would soft-reboot the nRF after
	 * ~4 min of broker-only outage, which fixes nothing. mqtt_fail_streak
	 * already handles broker-side escalation (eventual AT+CFUN=1,1) with its
	 * own longer threshold; let it own that lane. */
	if (mdm_pdp_alive) {
		LOG_INF("modem: FAULT (broker only, pdp alive) — backing off 5s");
	} else {
		consecutive_faults++;
		LOG_WRN("modem: FAULT (consecutive=%d) — backing off 5s",
		        consecutive_faults);
	}
	k_msleep(5000);
	if (!start_requested) return MDM_OFF;

	/* Last-resort self-heal: if we've been faulting for many minutes the
	 * nRF-side state (UART driver, rx parser, modem stack) may be wedged in a
	 * way the state machine can't clear (we've seen cases where only pressing
	 * the physical RESET button recovered). Soft-reboot is the software
	 * equivalent — pulled the plug ourselves rather than wait for a human. */
	/* Each cycle is ~50s (5s backoff + 16s probe + 13s pulse wait + 15s probe),
	 * so 5 failures ~ 4min — long enough that ordinary transients recover via
	 * PWRKEY toggling first, short enough that a genuinely wedged nRF state
	 * (UART driver, rx parser) gets cleared before the user reaches for the
	 * physical reset button. */
	if (consecutive_faults >= 7) {
		LOG_WRN("modem: %d consecutive faults — soft-rebooting nRF",
		        consecutive_faults);
		k_msleep(100);   /* let the log line drain */
		sys_reboot(SYS_REBOOT_COLD);
	}

	/* If the data link is still up (the broker-down case), skip the full
	 * AT/registration re-walk and retry just the MQTT layer — lighter and
	 * faster while we wait for the broker to come back. */
	if (mdm_pdp_alive) return MDM_MQTT_OPEN;

	/* Otherwise route back through BOOT_PWR. It probes AT before pulsing, so
	 * retries are self-correcting: if the module is alive it skips straight
	 * on; if it's truly off (e.g. an earlier stray pulse shut it down) the
	 * probe fails and it gets pulsed back on. */
	return MDM_BOOT_PWR;
}

static void modem_thread(void)
{
	cfg_load();
	gpio_pin_configure_dt(&mdm_pwr, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&mdm_rst, GPIO_OUTPUT_INACTIVE);
	(void)uart_start_rx();

	/* Autostart: bring the LTE link up on boot without waiting for a
	 * `modem start` console command — the receiver runs headless in the
	 * field (no laptop). Short settle delay lets ESB + USB CDC enumerate
	 * first. `modem stop` still halts it (start_requested=false). */
	k_msleep(2000);
	start_requested = true;
	LOG_INF("modem: autostart");

	while (1) {
		mdm_state_t cur = state;
		mdm_state_t next;
		switch (cur) {
		case MDM_OFF:        next = step_off();        break;
		case MDM_BOOT_PWR:   next = step_boot_pwr();   break;
		case MDM_AT_PROBE:   next = step_at_probe();   break;
		case MDM_AT_CONFIG:  next = step_at_config();  break;
		case MDM_SIM_WAIT:   next = step_sim_wait();   break;
		case MDM_NET_REG:    next = step_net_reg();    break;
		case MDM_PDP_ACT:    next = step_pdp_act();    break;
		case MDM_MQTT_OPEN:  next = step_mqtt_open();  break;
		case MDM_MQTT_CONN:  next = step_mqtt_conn();  break;
		case MDM_READY:      next = step_ready();      break;
		case MDM_FAULT:
		default:             next = step_fault();      break;
		}
		if (next != cur) {
			LOG_INF("modem: state %s -> %s",
			        state_names[cur], state_names[next]);
		}
		state = next;
	}
}
K_THREAD_DEFINE(modem_tid, 2048, modem_thread, NULL, NULL, NULL, 7, 0, 0);

/* ---- Console hooks --------------------------------------------------- */

void modem_console_status(void)
{
	printk("modem state: %s\n", modem_state_string());
	printk("apn   : %s\n", cfg.apn[0] ? cfg.apn : "(unset)");
	printk("broker: %s:%u\n", cfg.host[0] ? cfg.host : "(unset)", cfg.port);
	printk("client: %s\n", cfg.client[0] ? cfg.client : "(unset)");
	printk("topic : %s\n", cfg.topic[0] ? cfg.topic : "rideform/0/frame");
	printk("auth  : %s\n", cfg.user[0] ? "yes" : "no");
}

void modem_console_at(const char *cmd)
{
	if (!cmd || !*cmd) return;
	at_send_line(cmd);
	char line[MDM_LINE_MAX];
	int64_t deadline = k_uptime_get() + 2000;
	while (k_uptime_get() < deadline) {
		int err = k_msgq_get(&line_msgq, line,
			K_MSEC(deadline - k_uptime_get()));
		if (err) break;
		printk("%s\n", line);
	}
}

#define COPY_FIELD(dst, src) do { \
	memset((dst), 0, sizeof(dst));                                 \
	if ((src)) strncpy((dst), (src), sizeof(dst) - 1);             \
} while (0)

void modem_console_set_apn(const char *s)
{
	COPY_FIELD(cfg.apn, s);
	sys_write(MDM_NVS_APN, NULL, cfg.apn, sizeof(cfg.apn));
}
void modem_console_set_broker(const char *host, uint16_t port)
{
	COPY_FIELD(cfg.host, host);
	cfg.port = port ? port : 1883;
	sys_write(MDM_NVS_MQTT_HOST, NULL, cfg.host, sizeof(cfg.host));
	sys_write(MDM_NVS_MQTT_PORT, NULL, &cfg.port, sizeof(cfg.port));
}
void modem_console_set_client(const char *s)
{
	COPY_FIELD(cfg.client, s);
	sys_write(MDM_NVS_MQTT_CLIENT, NULL, cfg.client, sizeof(cfg.client));
}
void modem_console_set_topic(const char *s)
{
	COPY_FIELD(cfg.topic, s);
	sys_write(MDM_NVS_MQTT_TOPIC, NULL, cfg.topic, sizeof(cfg.topic));
}
void modem_console_set_user(const char *s)
{
	COPY_FIELD(cfg.user, s);
	sys_write(MDM_NVS_MQTT_USER, NULL, cfg.user, sizeof(cfg.user));
}
void modem_console_set_pass(const char *s)
{
	COPY_FIELD(cfg.pass, s);
	sys_write(MDM_NVS_MQTT_PASS, NULL, cfg.pass, sizeof(cfg.pass));
}

#endif /* MDM_AVAILABLE */
