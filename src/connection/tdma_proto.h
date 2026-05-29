/*
    RFT TDMA wire protocol (shared between tracker and receiver).

    This header MUST be kept identical between:
      apps/v2/firmware/src/connection/tdma_proto.h          (tracker side)
      apps/v2/firmware-receiver/src/connection/tdma_proto.h (receiver side)

    Replaces the ALOHA blast-and-collide scheme. Trackers stay PRX-silent
    until they receive a PKT_TDMA_BEACON from the receiver, then schedule
    their own TX at:

        tx_time = beacon_rx_time + head_offset_us + my_slot_index * slot_stride_us

    Missing a beacon → tracker stays silent that cycle (no extrapolation).
    Next beacon re-anchors. See apps/v2/PLAN_tdma.md for full design.
*/
#ifndef RFT_TDMA_PROTO_H
#define RFT_TDMA_PROTO_H

#include <stdint.h>

/* Number of TDMA slots per cycle. This is the count of trackers we're
 * willing to schedule; the receiver still supports MAX_TRACKERS=256 in
 * its pairing table, but only the first TDMA_NUM_SLOTS get a slot —
 * beyond that you'd be scheduling into sub-µs gaps and lose. Match
 * the Deck's MAX_SENSORS=10. */
#define TDMA_NUM_SLOTS         10

/* Reserved packet markers (data[0]). Existing markers 0x00..0x09 belong to
 * the tracker→receiver pkt0..pkt9 series; the 0xFx range is for
 * receiver→tracker control. */
#define PKT_TDMA_BEACON       0xFB   /* TIMING ACK byte[0] marker (legacy
                                      * name from the abandoned PTX-beacon
                                      * design — kept for the constant value) */

/* Beacon payload format. Single 16-byte ESB packet, fits in one TX.
 * Trackers read this on every beacon and re-anchor their slot timing —
 * no further state is required. */
struct __attribute__((packed)) tdma_beacon {
    uint8_t  marker;          /* = PKT_TDMA_BEACON (0xFB) */
    uint8_t  flags;           /* bit0 reserved for re-pair hint; rest unused */
    uint16_t head_offset_us;  /* wait this much after beacon RX before slot 0
                               * (accounts for beacon TX duration + receiver
                               * PTX→PRX switch + safety margin). 1000 µs
                               * is the default — enough for ~150 µs beacon +
                               * ~150 µs ramp + ~700 µs safety. */
    uint16_t slot_stride_us;  /* per-tracker offset:
                               *   tx_time = rx_time + head_offset
                               *             + my_slot_index * stride */
    uint32_t cycle_counter;   /* monotonic, increments on every beacon.
                               * Used by trackers to:
                               *  - rotate housekeeping packet types
                               *    (e.g. mag-d every 30 cycles, battery
                               *     every 300 cycles) without coordination
                               *  - detect drift (huge jumps = missed cycles) */
    uint8_t  num_slots;       /* = MAX_SENSORS (10). Informational — tracker
                               * doesn't need this to schedule its own TX. */
    uint8_t  reserved[5];     /* future use; transmit as zero, RX ignore */
};

/* Compile-time size check — must equal one ESB payload. */
_Static_assert(sizeof(struct tdma_beacon) == 16,
               "tdma_beacon must fit in one 16-byte ESB payload");

/* Default values used by the receiver to seed the beacon fields. The
 * head_offset is rate-independent (it sizes the receiver's PTX→PRX
 * settling window). Stride and cycle are computed per-rate by the
 * receiver — see tdma_compute_stride() in the receiver's tdma.c. */
#define TDMA_DEFAULT_HEAD_OFFSET_US  1000u
#define TDMA_DEFAULT_TAIL_GUARD_PCT  5u    /* of cycle period */

/* Recommended housekeeping cadences, in beacon cycles. A tracker at
 * slot_index I sends the housekeeping packet when
 *   (cycle_counter % period) == I
 * which staggers across the 10 trackers so they don't all stuff the
 * same low-priority packet into the same cycle. At 30 Hz beacon rate:
 *   MAG_D    every  30 cycles = 1 Hz   (matches current 1Hz behaviour)
 *   BATTERY  every 300 cycles = ~0.1 Hz (~10 s, was 1 Hz, slowed because
 *                                        we don't need it that often) */
#define TDMA_HOUSEKEEPING_PERIOD_MAG_D     30u
#define TDMA_HOUSEKEEPING_PERIOD_BATTERY   300u

#endif  /* RFT_TDMA_PROTO_H */
