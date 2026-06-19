/*
 * RFT: CDC ACM packet forwarder API.
 *
 * Pumps the same 16-byte SlimeVR HID-shape packets through a CDC ACM
 * endpoint, for Android hosts that can't easily use USB HID.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef RFT_CDC_FORWARD_H
#define RFT_CDC_FORWARD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Queue one 16-byte packet for transmission on the forward CDC port.
 * Safe to call from any context. Drops if the host isn't asserting DTR,
 * forwarding is disabled, or the internal ring buffer is full. */
void cdc_forward_write_packet(const uint8_t *data);

bool cdc_forward_is_enabled(void);
void cdc_forward_set_enabled(bool on);

#ifdef __cplusplus
}
#endif

#endif /* RFT_CDC_FORWARD_H */
