/*
 * RFT: BG770A-GL modem driver — public API.
 *
 * The Zephyr modem subsystem doesn't fit BG770A-GL's CMUX dialect, so we
 * drive AT directly over UART1 and walk the state machine ourselves.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef RFT_MODEM_H
#define RFT_MODEM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* NVS keys, extending system/system.h's range. STORED_ADDR_0 occupies
 * 3..258 so we start at 260 to leave a small gap. */
#define MDM_NVS_BASE          260
#define MDM_NVS_APN           (MDM_NVS_BASE + 0)
#define MDM_NVS_MQTT_HOST     (MDM_NVS_BASE + 1)
#define MDM_NVS_MQTT_PORT     (MDM_NVS_BASE + 2)
#define MDM_NVS_MQTT_CLIENT   (MDM_NVS_BASE + 3)
#define MDM_NVS_MQTT_TOPIC    (MDM_NVS_BASE + 4)
#define MDM_NVS_MQTT_USER     (MDM_NVS_BASE + 5)
#define MDM_NVS_MQTT_PASS     (MDM_NVS_BASE + 6)

#define MDM_APN_MAX        32
#define MDM_HOST_MAX       64
#define MDM_CLIENT_MAX     32
#define MDM_TOPIC_MAX      64
#define MDM_USER_MAX       32
#define MDM_PASS_MAX       32

/* Lifecycle. modem_init() runs from SYS_INIT; the worker stays parked in
 * MDM_OFF until modem_request_start() is invoked (typically once settings
 * have been provisioned). */
void modem_request_start(void);
void modem_request_stop(void);

/* Human-readable current state, for `modem status`. */
const char *modem_state_string(void);

/* Enqueue a 16-byte tracker packet for aggregated MQTT publish. The worker
 * batches packets at 30 Hz into one QMTPUB. Returns false if the aggregation
 * ring is full (caller drops). */
bool modem_enqueue_tracker_packet(const uint8_t *pkt16);

/* Console plumbing — invoked from src/console.c. */
void modem_console_status(void);
void modem_console_at(const char *cmd);     /* raw AT passthrough */
void modem_console_set_apn(const char *s);
void modem_console_set_broker(const char *host, uint16_t port);
void modem_console_set_client(const char *s);
void modem_console_set_topic(const char *s);
void modem_console_set_user(const char *s); /* "" clears */
void modem_console_set_pass(const char *s);

/* Live publish-tuning knobs (RAM only) for ceiling-probing experiments.
 *   pace: min ms between publish attempts (lower = push harder).
 *   ptmo: ms to wait for the QMTPUB ">" prompt before declaring a miss. */
void modem_set_pace(int ms);
void modem_set_ptmo(int ms);
int  modem_get_pace(void);
int  modem_get_ptmo(void);

#endif /* RFT_MODEM_H */
