/*
	RFT mag-cal command queue (receiver-side).

	The ESB ACK FIFO is broadcast-shared (depth 1, consumed by whichever
	tracker TXes next on pipe 1), so there are two distinct command paths:

	1. **Global flags** in byte [2]. Replicated to every paired tracker
	   automatically because every ACK delivers the current state. Used
	   for state that's the same across all trackers — currently just
	   "stream raw mag" on/off. Toggle is monolithic: ALL trackers stream
	   together, ALL stop together.

	2. **Targeted commands** in bytes [3..11]. One specific tracker
	   addressed by ID. Used for one-shot per-tracker commands like
	   SET_MAG_BIAS. Reliability comes from RFT_CMD_RETX_COUNT
	   re-transmissions of the same payload (since broadcast ACK FIFO
	   means random tracker consumes each write — only the target
	   tracker actually acts).

	Wire format on the sync packet (12 bytes):
	  [0..1]  : LED clock (existing, big-endian uint16)
	  [2]     : global flags (RFT_FLAG_*)
	  [3]     : target tracker ID (0xFF = no targeted command this slot)
	  [4]     : command type (RFT_CMD_*)
	  [5..10] : 6 bytes payload (command-specific)
	  [11]    : XOR checksum of bytes [2..10]
*/
#ifndef RFT_CMD_QUEUE_H
#define RFT_CMD_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define RFT_CMD_QUEUE_DEPTH 4

/* Global flags (byte [2]) — broadcast to every tracker on every sync */
#define RFT_FLAG_STREAM_RAW_MAG (1u << 0)  /* All paired trackers stream
                                            * packet 8 (raw_mag + bias)
                                            * while this bit is set. */

/* Targeted command types (byte [4]) — must match tracker firmware */
#define RFT_CMD_NONE           0
#define RFT_CMD_SET_MAG_BIAS   1  /* payload: 3 × int16 Q11 (Gauss) */
#define RFT_CMD_CLEAR_MAG_BIAS 2  /* payload: zeros */
#define RFT_CMD_MAG_RECAL      3  /* payload: zeros (RAM-only reset) */
/* RFT_CMD_STREAM_RAW_MAG (=4) was retired — moved to RFT_FLAG_STREAM_RAW_MAG.
 * Don't reuse the value for a different command until tracker firmware is
 * updated past commit a04f3a6 to be safe. */
#define RFT_CMD_SET_MAX_RATE_HZ 5 /* payload: uint8 rate Hz in [0..255],
                                   * 0 means "back to firmware default".
                                   * RAM only on tracker side — a power
                                   * cycle reverts to the built-in rate. */

#define RFT_CMD_NO_TARGET     0xFF

typedef struct {
	uint8_t type;       /* RFT_CMD_* */
	uint8_t data[6];    /* command-specific */
} rft_cmd_t;

/* Push a command for tracker `id`. Returns true on success, false if queue full
 * or id out of range. */
bool rft_cmd_push(uint8_t id, const rft_cmd_t *cmd);

/* Peek the next pending command using round-robin across tracker IDs.
 * Returns the tracker ID and fills `cmd_out` if a command is pending.
 * Returns RFT_CMD_NO_TARGET (0xFF) if no commands pending anywhere.
 *
 * Does NOT remove from queue. Caller must call rft_cmd_consume() after
 * a successful TX to decrement the retry counter / drop the entry.
 * If TX failed (e.g. ACK FIFO -ENOMEM), don't call consume so the same
 * cmd will be peeked again next tick. */
uint8_t rft_cmd_peek_next(rft_cmd_t *cmd_out);

/* Mark the most recently peeked command as transmitted once. Decrements
 * the retry counter; removes entry when counter reaches 0 and advances
 * the round-robin cursor to the next tracker. */
void rft_cmd_consume(uint8_t target_id);

/* Total times a command is emitted into the broadcast-shared ACK FIFO
 * before being dropped. The ACK FIFO is consumed by whichever tracker
 * TXes next on pipe 1 — only the target tracker actually executes the
 * cmd, so with N paired trackers we need ~N+ retries for high reliability.
 * 30 retries with 5 trackers ⇒ delivery probability ≥ 99.9%.
 * 50 Hz refresh × 30 retries = 600 ms per command. */
#define RFT_CMD_RETX_COUNT 30

/* Drop all pending commands for a given tracker (e.g., on unpair). */
void rft_cmd_clear(uint8_t id);

/* Global flag accessors. The flag byte is broadcast to every tracker on
 * every sync packet — flipping it via rft_set_global_flags() takes effect
 * on all paired trackers within ~20ms (one ACK round). No retransmit
 * needed because the value is replicated continuously. */
uint8_t rft_get_global_flags(void);
void rft_set_global_flags(uint8_t flags);
void rft_set_global_flag(uint8_t flag, bool on);

/* Pack the sync packet payload at [2..11].
 *   out[0]    = global flags (RFT_FLAG_*)
 *   out[1]    = target tracker ID (RFT_CMD_NO_TARGET = no targeted cmd)
 *   out[2]    = command type (ignored if target == NO_TARGET)
 *   out[3..8] = 6B command payload
 *   out[9]    = XOR checksum of out[0..8]
 */
void rft_cmd_pack(uint8_t flags, uint8_t target_id, const rft_cmd_t *cmd,
                  uint8_t out[10]);

#endif
