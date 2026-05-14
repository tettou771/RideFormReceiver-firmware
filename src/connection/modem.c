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

#else /* MDM_AVAILABLE */

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

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
					char tmp[MDM_LINE_MAX];
					memcpy(tmp, buf, pos + 1);
					(void)k_msgq_put(&line_msgq, tmp, K_NO_WAIT);
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

/* ---- Tracker packet aggregation -------------------------------------- */

#define MDM_PKT_BYTES   16
#define MDM_PKT_RING    256  /* fits ~33ms of 8 trackers @ ~50pps each */

static uint8_t agg_buf[MDM_PKT_RING][MDM_PKT_BYTES];
static atomic_t agg_w = ATOMIC_INIT(0);
static atomic_t agg_r = ATOMIC_INIT(0);

bool modem_enqueue_tracker_packet(const uint8_t *pkt16)
{
	int w = atomic_get(&agg_w);
	int r = atomic_get(&agg_r);
	int next = (w + 1) % MDM_PKT_RING;
	if (next == r) return false; /* full, drop */
	memcpy(agg_buf[w], pkt16, MDM_PKT_BYTES);
	atomic_set(&agg_w, next);
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

/* BG770A's PWRKEY is a toggle button: pulse while OFF turns it ON,
 * pulse while ON commands a shutdown. So we have to pulse it AT MOST
 * once per nRF52 boot — subsequent FAULT->BOOT_PWR retries (e.g. AT
 * timeout while the module is still finishing its own boot) must NOT
 * pulse again or we'd shut the modem down.
 *
 * Reset only happens via SoC reboot (which power-cycles this flag),
 * so once true it stays true for the life of this firmware run. */
static bool pwrkey_pulsed = false;

/* Tracks consecutive entries into FAULT without an intervening success.
 * If we stack up too many we suspect the BG770A is actually off (despite
 * our latch) — could happen if a previous run's PWRKEY pulse landed while
 * it was on and shut it down, leaving us stranded forever. Clear the
 * latch so the next BOOT_PWR re-pulses, giving us a way out. */
static int consecutive_faults = 0;

static mdm_state_t step_boot_pwr(void)
{
	if (!pwrkey_pulsed) {
		LOG_INF("modem: pulsing PWRKEY (first start)");
		gpio_pin_set_dt(&mdm_pwr, 1);
		k_msleep(700);
		gpio_pin_set_dt(&mdm_pwr, 0);
		pwrkey_pulsed = true;
		/* Quectel datasheet: ~13s from PWRKEY release to URC "RDY" on
		 * a cold boot. Sleep the worst case so AT_PROBE sees a ready
		 * module, then drain whatever URC sits in the line buffer. */
		k_msleep(13000);
		(void)at_wait_prefix("RDY", NULL, 0, 500);
	} else {
		LOG_INF("modem: PWRKEY already pulsed this run, skipping");
	}
	return MDM_AT_PROBE;
}

static mdm_state_t step_at_probe(void)
{
	for (int i = 0; i < 5; i++) {
		at_send_line("AT");
		if (at_wait_prefix("OK", NULL, 0, 1000) == 0) {
			consecutive_faults = 0;  /* AT round-trip ok, healthy */
			return MDM_AT_CONFIG;
		}
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
	at_send_line("ATE0");
	(void)at_wait_prefix("OK", NULL, 0, 500);
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

static mdm_state_t step_mqtt_open(void)
{
	if (!cfg.host[0]) {
		LOG_WRN("modem: MQTT host not configured");
		k_msleep(2000);
		return MDM_MQTT_OPEN;
	}

	char buf[160];
	char line[MDM_LINE_MAX];

	/* First confirm the PDP context is really up by re-asking — if QIACT
	 * stuck without an IP, everything below silently fails. */
	at_send_line("AT+QIACT?");
	if (at_wait_prefix("+QIACT:", line, sizeof(line), 3000) == 0) {
		LOG_INF("modem: %s", line);
	} else {
		LOG_WRN("modem: no PDP context active");
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
	return MDM_FAULT;
}

static mdm_state_t step_mqtt_conn(void)
{
	char buf[160];
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
		return MDM_READY;
	}
	LOG_WRN("modem: QMTCONN failed");
	return MDM_FAULT;
}

/* Drain aggregation ring and publish via QMTPUB binary mode.
 *
 * Critical-path timing — this is called from step_ready ~30Hz. Every ms
 * here delays the next publish, so all the waits are intentionally short
 * and any failure just drops the current frame and lets the next one go. */
static void publish_frame(void)
{
	int w = atomic_get(&agg_w);
	int r = atomic_get(&agg_r);
	if (w == r) return;
	int count = (w - r + MDM_PKT_RING) % MDM_PKT_RING;
	int payload_len = count * MDM_PKT_BYTES;

	const char *topic = cfg.topic[0] ? cfg.topic : "rideform/0/frame";
	char hdr[160];
	int n = snprintf(hdr, sizeof(hdr),
		"AT+QMTPUB=0,0,0,0,\"%s\",%d\r", topic, payload_len);
	if (n <= 0) return;
	at_send_raw((const uint8_t *)hdr, n);

	/* Diagnostic counters — rate-limited to one log per second so they
	 * don't drown the console at 30 Hz. */
	static int64_t s_last_log_ms = 0;
	static int s_pub_ok = 0, s_pub_prompt_miss = 0, s_pub_ack_miss = 0;
	static int s_pkts_sent = 0;

	/* Wait for the ">" prompt — emitted by BG770A right before it expects
	 * payload bytes. rx_line_thread emits ">" as its own line so we can
	 * just wait_prefix on it. */
	if (at_wait_prefix(">", NULL, 0, 500) != 0) {
		s_pub_prompt_miss++;
		/* Drain whatever did arrive in the prompt window — usually
		 * "ERROR" when the MQTT session is gone, "+CME ERROR: <n>"
		 * for command-level failures, etc. Tells us *why* prompt
		 * didn't show. */
		char line[MDM_LINE_MAX];
		int drained = 0;
		while (k_msgq_get(&line_msgq, line, K_NO_WAIT) == 0 && drained < 5) {
			LOG_INF("modem: pub-miss rsp: %s", line);
			drained++;
		}
		atomic_set(&agg_r, w);
		goto maybe_log;
	}

	int idx = r;
	for (int i = 0; i < count; i++) {
		at_send_raw(agg_buf[idx], MDM_PKT_BYTES);
		idx = (idx + 1) % MDM_PKT_RING;
	}
	atomic_set(&agg_r, w);

	/* Short ack wait; if PUBACK doesn't show, the next publish will
	 * still proceed (QoS 0 — we don't retry per-message anyway). */
	char ack[MDM_LINE_MAX];
	if (at_wait_prefix("+QMTPUB:", ack, sizeof(ack), 500) == 0) {
		s_pub_ok++;
		s_pkts_sent += count;
	} else {
		s_pub_ack_miss++;
		s_pkts_sent += count;
	}

maybe_log:
	{
		int64_t now = k_uptime_get();
		if (now - s_last_log_ms > 1000) {
			s_last_log_ms = now;
			LOG_INF("modem: pub ok=%d prompt_miss=%d ack_miss=%d pkts=%d",
			        s_pub_ok, s_pub_prompt_miss, s_pub_ack_miss, s_pkts_sent);
			s_pub_ok = 0;
			s_pub_prompt_miss = 0;
			s_pub_ack_miss = 0;
			s_pkts_sent = 0;
		}
	}
}

static mdm_state_t step_ready(void)
{
	publish_frame();
	k_msleep(33); /* ~30 fps */
	return MDM_READY;
}

static mdm_state_t step_fault(void)
{
	consecutive_faults++;
	LOG_WRN("modem: FAULT (consecutive=%d) — backing off 5s",
	        consecutive_faults);
	k_msleep(5000);
	if (!start_requested) return MDM_OFF;

	if (consecutive_faults >= 3) {
		LOG_WRN("modem: 3+ faults in a row, re-enabling PWRKEY pulse");
		pwrkey_pulsed = false;
		consecutive_faults = 0;
	}
	return MDM_BOOT_PWR;
}

static void modem_thread(void)
{
	cfg_load();
	gpio_pin_configure_dt(&mdm_pwr, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&mdm_rst, GPIO_OUTPUT_INACTIVE);
	(void)uart_start_rx();

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
