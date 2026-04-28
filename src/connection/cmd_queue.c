/*
	RFT mag-cal command queue (receiver-side).
	See cmd_queue.h for protocol description.
*/
#include "cmd_queue.h"
#include "../globals.h"
#include <string.h>
#include <zephyr/spinlock.h>

/* Per-tracker FIFO. Empty when head == tail. The head entry is what
 * pop_next returns; we leave it in place until tx_remaining counts down,
 * to keep retransmitting the same command into the broadcast-shared ACK
 * FIFO until the target tracker actually consumes it. */
typedef struct {
	rft_cmd_t entries[RFT_CMD_QUEUE_DEPTH];
	uint8_t head;     /* index of next pop */
	uint8_t tail;     /* index of next push */
	uint8_t count;    /* number of valid entries */
	uint8_t tx_remaining;  /* retries left for the head entry */
} cmd_fifo_t;

static cmd_fifo_t fifos[MAX_TRACKERS];
static uint16_t rr_cursor = 0;             /* round-robin start for next pop */
static volatile uint8_t global_flags = 0;  /* broadcast in every sync packet */
static struct k_spinlock lock;

uint8_t rft_get_global_flags(void)
{
	return global_flags;
}

void rft_set_global_flags(uint8_t flags)
{
	global_flags = flags;
}

void rft_set_global_flag(uint8_t flag, bool on)
{
	uint8_t v = global_flags;
	if (on) v |= flag;
	else    v &= (uint8_t)~flag;
	global_flags = v;
}

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

uint8_t rft_cmd_peek_next(rft_cmd_t *cmd_out)
{
	if (cmd_out == NULL) return RFT_CMD_NO_TARGET;
	k_spinlock_key_t k = k_spin_lock(&lock);
	for (int i = 0; i < MAX_TRACKERS; i++) {
		uint16_t idx = (rr_cursor + i) % MAX_TRACKERS;
		cmd_fifo_t *f = &fifos[idx];
		if (f->count > 0) {
			*cmd_out = f->entries[f->head];
			k_spin_unlock(&lock, k);
			return (uint8_t)idx;
		}
	}
	k_spin_unlock(&lock, k);
	return RFT_CMD_NO_TARGET;
}

void rft_cmd_consume(uint8_t target_id)
{
	if ((uint16_t)target_id >= MAX_TRACKERS) return;
	k_spinlock_key_t k = k_spin_lock(&lock);
	cmd_fifo_t *f = &fifos[target_id];
	if (f->count > 0) {
		if (f->tx_remaining == 0) {
			f->tx_remaining = RFT_CMD_RETX_COUNT;
		}
		f->tx_remaining--;
		if (f->tx_remaining == 0) {
			f->head = (f->head + 1) % RFT_CMD_QUEUE_DEPTH;
			f->count--;
			rr_cursor = (target_id + 1) % MAX_TRACKERS;
		} else {
			/* Move rr_cursor to interleave other trackers' commands */
			rr_cursor = (target_id + 1) % MAX_TRACKERS;
		}
	}
	k_spin_unlock(&lock, k);
}

void rft_cmd_clear(uint8_t id)
{
	if ((uint16_t)id >= MAX_TRACKERS) return;
	k_spinlock_key_t k = k_spin_lock(&lock);
	memset(&fifos[id], 0, sizeof(fifos[id]));
	k_spin_unlock(&lock, k);
}

void rft_cmd_pack(uint8_t flags, uint8_t target_id, const rft_cmd_t *cmd,
                  uint8_t out[10])
{
	memset(out, 0, 10);
	out[0] = flags;                 /* packet[2] global flags */
	if (target_id == RFT_CMD_NO_TARGET || cmd == NULL) {
		out[1] = RFT_CMD_NO_TARGET; /* packet[3] target = none */
	} else {
		out[1] = target_id;         /* packet[3] target */
		out[2] = cmd->type;         /* packet[4] cmd type */
		memcpy(&out[3], cmd->data, 6); /* packet[5..10] payload */
	}
	uint8_t xor = 0;
	for (int i = 0; i < 9; i++) xor ^= out[i];
	out[9] = xor;                   /* packet[11] checksum */
}
