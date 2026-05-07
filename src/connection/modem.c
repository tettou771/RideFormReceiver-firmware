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
 * "\r\n" both before and after). */
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
			if (pos < MDM_LINE_MAX - 1) buf[pos++] = b;
			else pos = 0; /* overflow: drop */
		}
	}
}
K_THREAD_DEFINE(modem_rx_tid, 1024, rx_line_thread, NULL, NULL, NULL, 6, 0, 0);

/* ---- AT TX ----------------------------------------------------------- */

static int at_send_raw(const uint8_t *data, size_t len)
{
	return uart_tx(uart_dev, data, len, SYS_FOREVER_US);
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

static mdm_state_t step_boot_pwr(void)
{
	/* PWRKEY: assert (active-low, open-drain) ~600ms then release. */
	gpio_pin_set_dt(&mdm_pwr, 1);
	k_msleep(700);
	gpio_pin_set_dt(&mdm_pwr, 0);
	LOG_INF("modem: PWRKEY pulsed; waiting for RDY...");
	/* Some firmwares emit "RDY" on the URC; some are silent. Either way,
	 * AT_PROBE will retry until AT/OK round-trips. */
	(void)at_wait_prefix("RDY", NULL, 0, 12000);
	return MDM_AT_PROBE;
}

static mdm_state_t step_at_probe(void)
{
	for (int i = 0; i < 5; i++) {
		at_send_line("AT");
		if (at_wait_prefix("OK", NULL, 0, 1000) == 0) return MDM_AT_CONFIG;
	}
	LOG_WRN("modem: AT probe timed out");
	return MDM_FAULT;
}

static mdm_state_t step_at_config(void)
{
	/* TODO(hardware): set ATE0, +CMEE=1, +QURCCFG="urcport","uart1",
	 *                 +CFUN=1, +QCFG="band",..., per Quectel app note. */
	at_send_line("ATE0");
	(void)at_wait_prefix("OK", NULL, 0, 500);
	at_send_line("AT+CMEE=1");
	(void)at_wait_prefix("OK", NULL, 0, 500);
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
	/* TODO(hardware): AT+CEREG=2; poll until <stat> ∈ {1,5}. */
	at_send_line("AT+CEREG?");
	char line[MDM_LINE_MAX];
	if (at_wait_prefix("+CEREG:", line, sizeof(line), 5000) == 0) {
		/* Parse "+CEREG: <n>,<stat>,..." — minimal sniff. */
		int n = 0, stat = 0;
		if (sscanf(line, "+CEREG: %d,%d", &n, &stat) >= 2 &&
		    (stat == 1 || stat == 5)) {
			return MDM_PDP_ACT;
		}
	}
	k_msleep(1000);
	return MDM_NET_REG;
}

static mdm_state_t step_pdp_act(void)
{
	/* TODO(hardware): AT+QICSGP=1,1,"<apn>",,,1 then AT+QIACT=1. */
	char buf[128];
	snprintf(buf, sizeof(buf),
		"AT+QICSGP=1,1,\"%s\",\"\",\"\",1",
		cfg.apn[0] ? cfg.apn : "");
	at_send_line(buf);
	(void)at_wait_prefix("OK", NULL, 0, 1000);
	at_send_line("AT+QIACT=1");
	if (at_wait_prefix("OK", NULL, 0, 30000) == 0) return MDM_MQTT_OPEN;
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
	char buf[128];
	snprintf(buf, sizeof(buf),
		"AT+QMTOPEN=0,\"%s\",%u", cfg.host, cfg.port);
	at_send_line(buf);
	if (at_wait_prefix("+QMTOPEN: 0,0", NULL, 0, 30000) == 0)
		return MDM_MQTT_CONN;
	LOG_WRN("modem: QMTOPEN failed");
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

/* Drain aggregation ring and publish via QMTPUB binary mode. */
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

	/* Wait for "> " prompt. BG770A emits it on its own line; our line
	 * splitter drops the empty line so we approximate by a fixed delay. */
	k_msleep(50);

	int idx = r;
	for (int i = 0; i < count; i++) {
		at_send_raw(agg_buf[idx], MDM_PKT_BYTES);
		idx = (idx + 1) % MDM_PKT_RING;
	}
	atomic_set(&agg_r, w);

	(void)at_wait_prefix("+QMTPUB:", NULL, 0, 5000);
}

static mdm_state_t step_ready(void)
{
	publish_frame();
	k_msleep(33); /* ~30 fps */
	return MDM_READY;
}

static mdm_state_t step_fault(void)
{
	LOG_WRN("modem: FAULT — backing off 5s before retry");
	k_msleep(5000);
	return start_requested ? MDM_BOOT_PWR : MDM_OFF;
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
