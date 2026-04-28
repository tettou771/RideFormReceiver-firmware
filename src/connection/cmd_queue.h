/*
	RFT mag-cal command queue (receiver-side).

	Trackers paired to the receiver are addressed by a small ID (0..MAX_TRACKERS-1).
	The receiver broadcasts a sync packet every ~3ms to all trackers (LED clock
	sync). We extend that packet to also carry one optional command targeting
	one specific tracker. Trackers ignore commands whose target ID doesn't match
	their own.

	Each tracker has a tiny FIFO of pending commands (RFT_CMD_QUEUE_DEPTH each).
	The TX path round-robins across tracker IDs to avoid one tracker starving
	others.

	Wire format on the sync packet (12 bytes):
	  [0..1] : LED clock (existing, big-endian uint16)
	  [2]    : target tracker ID (0xFF = no command this slot)
	  [3]    : command type (RFT_CMD_*)
	  [4..9] : 6 bytes payload (command-specific)
	  [10]   : reserved
	  [11]   : XOR checksum of bytes [2..10]
*/
#ifndef RFT_CMD_QUEUE_H
#define RFT_CMD_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define RFT_CMD_QUEUE_DEPTH 4

/* Command types — must match tracker firmware */
#define RFT_CMD_NONE          0
#define RFT_CMD_SET_MAG_BIAS  1  /* payload: 3 × int16 Q11 (Gauss) */
#define RFT_CMD_CLEAR_MAG_BIAS 2 /* payload: zeros */
#define RFT_CMD_MAG_RECAL     3  /* payload: zeros (RAM-only reset) */

#define RFT_CMD_NO_TARGET     0xFF

typedef struct {
	uint8_t type;       /* RFT_CMD_* */
	uint8_t data[6];    /* command-specific */
} rft_cmd_t;

/* Push a command for tracker `id`. Returns true on success, false if queue full
 * or id out of range. */
bool rft_cmd_push(uint8_t id, const rft_cmd_t *cmd);

/* Pop one pending command using round-robin across tracker IDs. Returns the
 * tracker ID and fills `cmd_out` if a command was popped. Returns
 * RFT_CMD_NO_TARGET (0xFF) if no commands pending anywhere. */
uint8_t rft_cmd_pop_next(rft_cmd_t *cmd_out);

/* Drop all pending commands for a given tracker (e.g., on unpair). */
void rft_cmd_clear(uint8_t id);

/* Pack a command into the sync packet payload at [2..11] given target id and
 * cmd. If target == RFT_CMD_NO_TARGET, fills with zeros. */
void rft_cmd_pack(uint8_t target_id, const rft_cmd_t *cmd, uint8_t out[10]);

#endif
