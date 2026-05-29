/*
    RFT TDMA scheduler — receiver side.

    Maintains the cycle clock and packs the TIMING ACK payload that
    trackers parse to anchor their slot timing. See tdma.h + PLAN_tdma.md.

    The 1 kHz refresh thread itself lives in timer.c (it replaces the
    legacy sync_thread and decides each tick whether to send a TIMING
    or CMD ACK via the helpers below).
*/
#include "globals.h"
#include "tdma.h"
#include "tdma_proto.h"
#include "cmd_queue.h"

#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

#define TDMA_DEFAULT_RATE_HZ  30
#define TDMA_MIN_RATE_HZ       1
#define TDMA_MAX_RATE_HZ     200

/* All cycle math is in microseconds. k_uptime_ticks() ticks at
 * CONFIG_SYS_CLOCK_TICKS_PER_SEC (32768 on nRF, so ~30.5 µs / tick).
 * Conversions go through k_ticks_to_us_floor64 / k_us_to_ticks_ceil64
 * which are inline and cheap. */
static uint8_t  current_rate_hz   = TDMA_DEFAULT_RATE_HZ;
static uint32_t cycle_period_us   = 1000000u / TDMA_DEFAULT_RATE_HZ;
static uint16_t slot_stride_us    = 0;   /* recomputed in recompute_geometry */
static uint16_t head_offset_us    = TDMA_DEFAULT_HEAD_OFFSET_US;

/* cycle_start_us in receiver's microsecond clock. Advances by
 * cycle_period_us at each cycle boundary. cycle_counter wraps freely. */
static uint64_t cycle_start_us    = 0;
static uint32_t cycle_counter     = 0;

/* For tdma_should_send_cmd_ack() — pace mag-flag refresh even with no
 * pending command. ~50 Hz refresh is plenty for a flag that only needs
 * to be authoritative within ~20 ms after a toggle. */
#define TDMA_CMD_REFRESH_TICKS 20  /* at 1 kHz refresh = every 20 ms */
static uint16_t cmd_refresh_tick  = 0;

static uint64_t now_us(void)
{
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

/* Compute head/stride for the current rate. Tail guard = 5 % of cycle.
 * stride = (T - head - tail_guard) / num_slots. */
static void recompute_geometry(void)
{
    const uint32_t T = cycle_period_us;
    const uint32_t tail = T * TDMA_DEFAULT_TAIL_GUARD_PCT / 100u;
    const uint32_t head = TDMA_DEFAULT_HEAD_OFFSET_US;
    const uint32_t avail = (T > head + tail) ? T - head - tail : 0;
    head_offset_us = (uint16_t)head;
    slot_stride_us = (uint16_t)(avail / TDMA_NUM_SLOTS);
    LOG_INF("geometry rate=%uHz T=%uus head=%uus stride=%uus tail=%uus",
            current_rate_hz, T, head_offset_us, slot_stride_us, tail);
}

void tdma_init(void)
{
    cycle_start_us = now_us();
    cycle_counter  = 0;
    recompute_geometry();
}

void tdma_set_rate_hz(uint8_t hz)
{
    if (hz == 0) hz = TDMA_DEFAULT_RATE_HZ;
    if (hz < TDMA_MIN_RATE_HZ) hz = TDMA_MIN_RATE_HZ;
    if (hz > TDMA_MAX_RATE_HZ) hz = TDMA_MAX_RATE_HZ;
    current_rate_hz = hz;
    cycle_period_us = 1000000u / hz;
    /* Reset cycle anchor at the change. Trackers will resync within
     * one cycle via fresh timing ACKs. */
    cycle_start_us = now_us();
    recompute_geometry();
}

uint8_t tdma_get_rate_hz(void) { return current_rate_hz; }

/* Advance cycle_start_us / cycle_counter past now_us so that
 * cycle_start_us <= now_us < cycle_start_us + cycle_period_us.
 * Called from tdma_pack_timing_ack() so the value is up-to-date at
 * the moment we write the ACK. */
static void advance_cycle_to_now(uint64_t now)
{
    if (now < cycle_start_us) return;  /* clock jumped backwards somehow */
    while (now - cycle_start_us >= cycle_period_us) {
        cycle_start_us += cycle_period_us;
        cycle_counter++;
    }
}

void tdma_pack_timing_ack(uint8_t out[12])
{
    uint64_t now = now_us();
    advance_cycle_to_now(now);
    uint64_t next_cycle = cycle_start_us + cycle_period_us;
    uint32_t us_until = (uint32_t)(next_cycle - now);
    if (us_until > 0xFFFFFFu) us_until = 0xFFFFFFu;  /* clamp to 24-bit */

    out[0]  = PKT_TDMA_BEACON;                       /* 0xFB marker */
    out[1]  = (uint8_t)( us_until        & 0xFF);    /* us_until lo */
    out[2]  = (uint8_t)((us_until >>  8) & 0xFF);
    out[3]  = (uint8_t)((us_until >> 16) & 0xFF);    /* us_until hi (24-bit) */
    out[4]  = (uint8_t)( cycle_counter        & 0xFF);
    out[5]  = (uint8_t)((cycle_counter >> 8)  & 0xFF);
    out[6]  = (uint8_t)( slot_stride_us       & 0xFF);
    out[7]  = (uint8_t)((slot_stride_us >> 8) & 0xFF);
    out[8]  = (uint8_t)(head_offset_us / 8u);        /* 8 µs steps */
    out[9]  = TDMA_NUM_SLOTS;                          /* informational */
    out[10] = 0;                                     /* reserved */
    uint8_t x = 0;
    for (int i = 0; i < 11; i++) x ^= out[i];
    out[11] = x;
}

bool tdma_should_send_cmd_ack(void)
{
    /* Peek without consuming — check whether a targeted cmd is queued. */
    rft_cmd_t scratch;
    if (rft_cmd_peek_next(&scratch) != RFT_CMD_NO_TARGET) return true;

    /* Heartbeat for the global flag (mag stream). Without this, a
     * tracker that hasn't done a TX in TDMA_CMD_REFRESH_TICKS won't see
     * a flag change until its next slot — fine in practice but cheap to
     * keep snappy. */
    cmd_refresh_tick++;
    if (cmd_refresh_tick >= TDMA_CMD_REFRESH_TICKS) {
        cmd_refresh_tick = 0;
        return true;
    }
    return false;
}
