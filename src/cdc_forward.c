/*
 * RFT: CDC ACM packet forwarder.
 *
 * Streams the same 16-byte SlimeVR HID-shape packets that go to the USB
 * HID interface out a dedicated CDC ACM endpoint, back-to-back, with no
 * framing or length prefix. Lets an Android host read packets via
 * usb-serial-for-android — Android's USB host API has no clean HID class
 * driver path without root.
 *
 * Build-time gate: requires a DT node labelled `cdc_acm_fwd` of type
 * zephyr,cdc-acm-uart (see boards/xiao_ble.overlay). If the current
 * board overlay does not define it, this file compiles to stubs.
 *
 * SPDX-License-Identifier: MIT
 */

#include "globals.h"
#include "cdc_forward.h"

#define CDC_FORWARD_NODE DT_NODELABEL(cdc_acm_fwd)

#if DT_NODE_HAS_STATUS(CDC_FORWARD_NODE, okay)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#include "system/system.h"

LOG_MODULE_REGISTER(cdc_fwd, LOG_LEVEL_INF);

/* NVS slot — high number, well clear of modem.h MDM_NVS_BASE (260..266). */
#define CDC_FWD_NVS_ENABLE  280

/* Stored encoding: 0=uninitialised (treat as default-on), 1=on, 2=off.
 * Keeps default-ON behaviour without paying the cost of writing on first
 * boot. */
#define CDC_FWD_STORED_ON   1
#define CDC_FWD_STORED_OFF  2

#define RING_BYTES  (16 * 64)  /* 64 frames of headroom */

RING_BUF_DECLARE(cdc_fwd_rb, RING_BYTES);

static const struct device *cdc_fwd_dev;
static atomic_t cdc_fwd_enabled = ATOMIC_INIT(1); /* default ON */
static atomic_t cdc_fwd_dtr     = ATOMIC_INIT(0);

static void cdc_fwd_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		/* Drain RX so a host that opened the port can't stall us. */
		if (uart_irq_rx_ready(dev)) {
			uint8_t scratch[32];
			(void)uart_fifo_read(dev, scratch, sizeof(scratch));
		}
		if (uart_irq_tx_ready(dev)) {
			uint8_t *buf;
			uint32_t claim = ring_buf_get_claim(&cdc_fwd_rb, &buf, 64);
			if (claim == 0) {
				uart_irq_tx_disable(dev);
				continue;
			}
			int sent = uart_fifo_fill(dev, buf, claim);
			if (sent < 0) sent = 0;
			ring_buf_get_finish(&cdc_fwd_rb, (uint32_t)sent);
			if (ring_buf_is_empty(&cdc_fwd_rb)) {
				uart_irq_tx_disable(dev);
			}
		}
	}
}

void cdc_forward_write_packet(const uint8_t *data)
{
	if (!atomic_get(&cdc_fwd_enabled)) return;
	if (!cdc_fwd_dev) return;
	if (!atomic_get(&cdc_fwd_dtr)) return; /* no host listening — drop */

	uint32_t put = ring_buf_put(&cdc_fwd_rb, data, 16);
	if (put != 16) {
		/* Backpressure: host isn't draining. Drop silently — the HID
		 * path stays authoritative. */
		return;
	}
	uart_irq_tx_enable(cdc_fwd_dev);
}

bool cdc_forward_is_enabled(void)
{
	return atomic_get(&cdc_fwd_enabled) != 0;
}

void cdc_forward_set_enabled(bool on)
{
	atomic_set(&cdc_fwd_enabled, on ? 1 : 0);
	uint8_t v = on ? CDC_FWD_STORED_ON : CDC_FWD_STORED_OFF;
	sys_write(CDC_FWD_NVS_ENABLE, NULL, &v, sizeof(v));
	if (!on && cdc_fwd_dev) {
		uart_irq_tx_disable(cdc_fwd_dev);
		ring_buf_reset(&cdc_fwd_rb);
	}
}

static void cdc_fwd_dtr_thread(void)
{
	/* Poll DTR so we don't fill the ring buffer when nothing is
	 * listening — keeps backpressure visible only when a real host is
	 * connected. */
	while (1) {
		if (cdc_fwd_dev) {
			uint32_t dtr = 0;
			(void)uart_line_ctrl_get(cdc_fwd_dev, UART_LINE_CTRL_DTR, &dtr);
			atomic_set(&cdc_fwd_dtr, dtr ? 1 : 0);
		}
		k_msleep(100);
	}
}

K_THREAD_DEFINE(cdc_fwd_dtr_tid, 512, cdc_fwd_dtr_thread, NULL, NULL, NULL,
		7, 0, 0);

static int cdc_forward_init(void)
{
	cdc_fwd_dev = DEVICE_DT_GET(CDC_FORWARD_NODE);
	if (!device_is_ready(cdc_fwd_dev)) {
		LOG_ERR("cdc_acm_fwd device not ready");
		cdc_fwd_dev = NULL;
		return -ENODEV;
	}

	uint8_t v = 0;
	sys_read(CDC_FWD_NVS_ENABLE, &v, sizeof(v));
	bool en = (v != CDC_FWD_STORED_OFF); /* default ON when uninitialised */
	atomic_set(&cdc_fwd_enabled, en ? 1 : 0);

	uart_irq_callback_set(cdc_fwd_dev, cdc_fwd_uart_isr);
	uart_irq_rx_enable(cdc_fwd_dev);

	LOG_INF("cdc_forward init: enabled=%d", (int)en);
	return 0;
}

SYS_INIT(cdc_forward_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#else /* CDC_FORWARD_NODE not enabled — stubs keep esb.c / hid.c linking. */

void cdc_forward_write_packet(const uint8_t *data) { (void)data; }
bool cdc_forward_is_enabled(void) { return false; }
void cdc_forward_set_enabled(bool on) { (void)on; }

#endif
