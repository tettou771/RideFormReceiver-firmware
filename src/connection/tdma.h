/*
    RFT TDMA scheduler (receiver side).

    Maintains the cycle clock (when the current cycle started, what
    cycle_counter it is, what the per-rate slot_stride is) and packs the
    TIMING ACK payload delivered to trackers via the ESB ACK FIFO. See
    PLAN_tdma.md for the design and the ACK byte[0]=0xFB marker.

    Replaces an unused upstream slimevr-receiver stub. None of its
    symbols are referenced anywhere else.
*/
#ifndef RFT_TDMA_H
#define RFT_TDMA_H

#include <stdint.h>
#include <stdbool.h>
#include "tdma_proto.h"   /* TDMA_NUM_SLOTS, PKT_TDMA_BEACON marker */

/* Bring up the cycle clock + 1 kHz refresh thread. Safe to call from
 * main() after esb is initialised. */
void tdma_init(void);

/* Switch the cycle rate. Recomputes head_offset/stride/cycle_period
 * from the new rate. hz==0 reverts to the firmware default (30 Hz).
 * Effective on the next refresh tick (≤1 ms). Called from the receiver
 * console `rate <hz>` and from the Deck-driven rate cmd. */
void tdma_set_rate_hz(uint8_t hz);
uint8_t tdma_get_rate_hz(void);

/* Pack a TIMING ACK payload at out[0..11] (12 bytes — must match
 * tx_payload_sync.length in esb.c). Layout:
 *   [0]   = 0xFB (TDMA TIMING marker — safe because the CMD ACK's
 *           byte[0] is the led_clock high byte and only reaches 0x0D)
 *   [1..3]= us_until_next_cycle_start (24-bit LE), refreshed at call
 *           time so the value the tracker reads is at most ~1 ms stale
 *   [4..5]= cycle_counter low 16 bits (LE) — drives housekeeping rotation
 *   [6..7]= slot_stride_us (16-bit LE)
 *   [8]   = head_offset_us / 8 (0..255 ⇒ 0..2040 µs in 8 µs steps)
 *   [9]   = num_slots (= MAX_SENSORS, informational)
 *   [10]  = reserved (0)
 *   [11]  = XOR checksum of [0..10]
 */
void tdma_pack_timing_ack(uint8_t out[12]);

/* True when the next ACK slot should carry a CMD instead of TIMING.
 * Current policy: when rft_cmd_peek_next() reports a queued cmd, OR
 * once every ~20 ticks (~20 ms at 1 kHz) so the global mag-stream
 * flag keeps refreshing on every tracker. */
bool tdma_should_send_cmd_ack(void);

#endif
