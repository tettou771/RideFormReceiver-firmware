/*
	RFT mag-cal command queue (receiver-side).
	See cmd_queue.h for protocol description.
*/
#include "cmd_queue.h"
#include "../globals.h"
#include <string.h>
#include <zephyr/spinlock.h>

/* Per-tracker FIFO. Empty when head == tail. */
typedef struct {
	rft_cmd_t entries[RFT_CMD_QUEUE_DEPTH];
	uint8_t head;     /* index of next pop */
	uint8_t tail;     /* index of next push */
	uint8_t count;    /* number of valid entries */
} cmd_fifo_t;

static cmd_fifo_t fifos[MAX_TRACKERS];
static uint16_t rr_cursor = 0;             /* round-robin start for next pop */
static struct k_spinlock lock;

bool rft_cmd_push(uint8_t id, const rft_cmd_t *cmd)
{
	if ((uint16_t)id >= MAX_TRACKERS || cmd == NULL) return false;
	k_spinlock_key_t k = k_spin_lock(&lock);
	cmd_fifo_t *f = &fifos[id];
	if (f->count >= RFT_CMD_QUEUE_DEPTH) {
		k_spin_unlock(&lock, k);
		return false;
	}
	f->entries[f->tail] = *cmd;
	f->tail = (f->tail + 1) % RFT_CMD_QUEUE_DEPTH;
	f->count++;
	k_spin_unlock(&lock, k);
	return true;
}

uint8_t rft_cmd_pop_next(rft_cmd_t *cmd_out)
{
	if (cmd_out == NULL) return RFT_CMD_NO_TARGET;
	k_spinlock_key_t k = k_spin_lock(&lock);
	for (int i = 0; i < MAX_TRACKERS; i++) {
		uint16_t idx = (rr_cursor + i) % MAX_TRACKERS;
		cmd_fifo_t *f = &fifos[idx];
		if (f->count > 0) {
			*cmd_out = f->entries[f->head];
			f->head = (f->head + 1) % RFT_CMD_QUEUE_DEPTH;
			f->count--;
			rr_cursor = (idx + 1) % MAX_TRACKERS;
			k_spin_unlock(&lock, k);
			return (uint8_t)idx;
		}
	}
	k_spin_unlock(&lock, k);
	return RFT_CMD_NO_TARGET;
}

void rft_cmd_clear(uint8_t id)
{
	if ((uint16_t)id >= MAX_TRACKERS) return;
	k_spinlock_key_t k = k_spin_lock(&lock);
	memset(&fifos[id], 0, sizeof(fifos[id]));
	k_spin_unlock(&lock, k);
}

void rft_cmd_pack(uint8_t target_id, const rft_cmd_t *cmd, uint8_t out[10])
{
	memset(out, 0, 10);
	if (target_id == RFT_CMD_NO_TARGET || cmd == NULL) {
		out[0] = RFT_CMD_NO_TARGET; /* target byte (slot index 2 in packet) */
		out[8] = 0;                 /* checksum */
		return;
	}
	out[0] = target_id;             /* packet[2] */
	out[1] = cmd->type;             /* packet[3] */
	memcpy(&out[2], cmd->data, 6);  /* packet[4..9] */
	out[8] = 0;                     /* packet[10] reserved */
	uint8_t xor = 0;
	for (int i = 0; i < 9; i++) xor ^= out[i];
	out[9] = xor;                   /* packet[11] checksum */
}
