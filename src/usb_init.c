/*
 * RFT: bring USB CDC ACM up on builds where HID is disabled.
 *
 * On the XIAO/dongle builds, hid.c::usb_init_thread is what eventually
 * calls usb_enable(). When HID is compiled out (Wio BG770A LTE build),
 * nothing else does — and CDC ACM never enumerates. This thread fills
 * that gap.
 *
 * SPDX-License-Identifier: MIT
 */
#include "globals.h"

#if !IS_ENABLED(CONFIG_USB_DEVICE_HID)

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(usb_init, LOG_LEVEL_INF);

static void rft_usb_init(void)
{
	int ret = usb_enable(NULL);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("usb_enable: %d", ret);
	}
}
K_THREAD_DEFINE(rft_usb_init_tid, 1024, rft_usb_init, NULL, NULL, NULL,
		7, 0, 0);

#endif /* !CONFIG_USB_DEVICE_HID */
