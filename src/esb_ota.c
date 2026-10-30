/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
/*
 * ESB OTA Firmware Update – Receiver Side (Relay)
 *
 * The receiver acts as a relay between the PC (USB HID) and trackers (ESB).
 * Supports parallel OTA for up to OTA_MAX_PARALLEL trackers with same firmware.
 *
 * Data flow:
 *   PC → HID OUT → esb_ota_relay_process_hid() → shared ring buffer
 *   Ring buffer → esb_ota_relay_fill_ack() → ESB ACK payload → Tracker
 *   Tracker → ESB status/info packets → esb_ota_relay_process_tracker_packet() → HID IN → PC
 *
 * Parallel OTA uses a seq-indexed ring buffer with automatic retransmission.
 * The tracker reports next_seq in every STATUS; the receiver looks up
 * ring[next_seq] and sends it. Lost ACKs cause the tracker to re-request
 * the same seq, triggering automatic retransmission without PC intervention.
 * Non-OTA trackers are suppressed (reduced poll rate) during OTA.
 */

#include "esb_ota.h"
#include "hid.h"
#include "globals.h"
#include "connection/esb.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(esb_ota_relay, LOG_LEVEL_INF);

/* ── OTA Seq-Indexed Ring Buffer ───────────────────────────────────────── */

/*
 * Seq-indexed sliding window for OTA data packets (shared by all targets).
 * Producer: HID read thread (via esb_ota_relay_process_hid)
 * Consumer: Radio ISR (via esb_ota_relay_fill_ack)
 *
 * ring[seq & RING_MASK] stores the DATA packet for that seq number.
 * ring_head: next seq to be written (monotonically increasing)
 * tracker[].next_seq: last confirmed next_seq from tracker STATUS
 *
 * Retransmission is automatic: when the tracker reports next_seq=N,
 * fill_ack sends ring[N & MASK]. If the ACK is lost, the tracker
 * re-sends the same next_seq and gets the same packet again.
 * No per-tracker read cursor (ring_tail) is needed.
 */
struct ota_tx_entry {
	uint8_t data[64];
	uint8_t length;
};

/* Per-tracker OTA state */
struct ota_per_tracker {
	bool participating;                  /* Is this tracker in the OTA session? */
	uint8_t status;                      /* Last status from this tracker */
	uint16_t next_seq;                   /* Last confirmed next_seq from STATUS */
	uint8_t pending_cmd_type;            /* 0 = none */
	uint8_t pending_cmd_data[64];
	uint8_t pending_cmd_len;
	uint32_t pending_cmd_set_ticks;      /* k_uptime_ticks() when cmd was queued */
	uint32_t pending_cmd_send_count;     /* Times sent in ACK (for diagnostics) */
	int64_t last_hid_forward_time;       /* Throttle per-tracker HID status */
};

#define OTA_ABORT_TOMBSTONE_MS 10000

static uint32_t ota_abort_expiry_ticks[MAX_TRACKERS];
#define OTA_ABORT_TOMBSTONE_TICKS k_ms_to_ticks_floor32(OTA_ABORT_TOMBSTONE_MS)
static bool ota_abort_valid[MAX_TRACKERS];

/* Read the clock before this lock; Zephyr's uptime reader has its own spinlock. */
static struct k_spinlock ota_relay_lock;

static struct {
	bool active;
	uint8_t num_targets;
	uint8_t target_ids[OTA_MAX_PARALLEL]; /* List of participating tracker IDs */

	/* Session parameters (set during first BEGIN) */
	uint32_t image_size;
	uint16_t total_packets;

	/* Shared seq-indexed ring buffer (same data for all targets) */
	struct ota_tx_entry ring[OTA_TX_RING_SIZE];
	uint32_t ring_head;                   /* Next seq to be written (by HID thread) */

	/* Per-tracker state (indexed by tracker_id, not target index) */
	struct ota_per_tracker tracker[MAX_TRACKERS];
} ota_relay = {
	.active = false,
	.num_targets = 0,
	.ring_head = 0,
};

/* ── FW_INFO deferred send (ISR → thread context) ───────────────── */

static struct {
	uint8_t data[66];
	uint8_t len;
	uint8_t tracker_id;
	bool pending;
} fw_info_pending;

static void fw_info_send_work_handler(struct k_work *work);
static K_WORK_DEFINE(fw_info_send_work, fw_info_send_work_handler);

static void fw_info_send_work_handler(struct k_work *work)
{
	if (!fw_info_pending.pending) {
		return;
	}

	uint8_t *data = fw_info_pending.data;
	uint8_t len = fw_info_pending.len;
	uint8_t tracker_id = fw_info_pending.tracker_id;

	LOG_INF("OTA: Sending FW_INFO chunks for tracker %u", tracker_id);

	for (int chunk = 0; chunk < 6; chunk++) {
		uint8_t hid_report[16] = {0};
		hid_report[0] = HID_OTA_FW_INFO;
		hid_report[1] = tracker_id;
		hid_report[2] = chunk;
		size_t offset = 2 + chunk * 13;
		size_t copy_len = MIN(len - offset, 13);
		if (offset < len && copy_len > 0) {
			memcpy(&hid_report[3], &data[offset], copy_len);
		}
		hid_write_packet_n(hid_report, 0);
		/* Small delay between chunks to let HID FIFO drain */
		k_msleep(2);
	}

	fw_info_pending.pending = false;
}

/* ── Public API ──────────────────────────────────────────────────── */

bool esb_ota_relay_is_active(void)
{
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	bool active = ota_relay.active;
	k_spin_unlock(&ota_relay_lock, key);
	return active;
}

bool esb_ota_relay_is_target(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return false;
	}
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	bool target = ota_relay.active && ota_relay.tracker[tracker_id].participating;
	k_spin_unlock(&ota_relay_lock, key);
	return target;
}

/* ── Ring Buffer Helpers ─────────────────────────────────────────── */

/*
 * Compute the minimum next_seq across all participating trackers.
 * Data below this seq is fully consumed by all trackers.
 * The producer cannot write beyond min_seq + RING_SIZE.
 */
static uint32_t ring_seq_min_locked(void)
{
	uint32_t min_seq = ota_relay.ring_head;
	for (uint8_t i = 0; i < ota_relay.num_targets; i++) {
		uint8_t id = ota_relay.target_ids[i];
		uint32_t s = ota_relay.tracker[id].next_seq;
		if ((int32_t)(s - min_seq) < 0) {
			min_seq = s;
		}
	}
	return min_seq;
}

static uint32_t ring_count_locked(void)
{
	return ota_relay.ring_head - ring_seq_min_locked();
}

static bool ota_abort_pending_locked(uint8_t tracker_id, uint32_t now)
{
	bool valid = ota_abort_valid[tracker_id];
	if (valid && (int32_t)(ota_abort_expiry_ticks[tracker_id] - now) <= 0) {
		ota_abort_valid[tracker_id] = false;
		valid = false;
	}
	return valid;
}

/* ── Tracker Suppression ─────────────────────────────────────────── */

/*
 * Suppress non-OTA trackers to free radio bandwidth during OTA.
 * Suppressed trackers reduce their poll rate to ~1 Hz.
 */
static void suppress_non_ota_trackers(void)
{
	uint8_t ids[MAX_TRACKERS];
	uint8_t count = 0;
	unsigned int irq_key = irq_lock();
	uint32_t now = k_uptime_ticks();
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
		if (!ota_relay.tracker[i].participating && !ota_abort_pending_locked(i, now)) {
			ids[count++] = i;
		}
	}
	k_spin_unlock(&ota_relay_lock, key);
	irq_unlock(irq_key);
	for (uint8_t i = 0; i < count; i++) {
		esb_send_remote_command(ids[i], ESB_PONG_FLAG_OTA_SUPPRESS);
	}
}

static void unsuppress_all_trackers(void)
{
	uint8_t ids[MAX_TRACKERS];
	uint8_t count = 0;
	unsigned int irq_key = irq_lock();
	uint32_t now = k_uptime_ticks();
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
		if (!ota_abort_pending_locked(i, now)) {
			ids[count++] = i;
		}
	}
	k_spin_unlock(&ota_relay_lock, key);
	irq_unlock(irq_key);
	for (uint8_t i = 0; i < count; i++) {
		esb_send_remote_command(ids[i], ESB_PONG_FLAG_OTA_UNSUPPRESS);
	}
}

/* ── Session Management ──────────────────────────────────────────── */

static void reset_session_locked(void)
{
	ota_relay.active = false;
	ota_relay.num_targets = 0;
	ota_relay.ring_head = 0;
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		ota_relay.tracker[i].participating = false;
		ota_relay.tracker[i].status = OTA_STATUS_IDLE;
		ota_relay.tracker[i].next_seq = 0;
		ota_relay.tracker[i].pending_cmd_type = 0;
		ota_relay.tracker[i].pending_cmd_set_ticks = 0;
		ota_relay.tracker[i].pending_cmd_send_count = 0;
		ota_relay.tracker[i].last_hid_forward_time = 0;
	}
}

/* Caller holds ota_relay_lock. */
static bool remove_target_locked(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS || !ota_relay.tracker[tracker_id].participating) {
		return false;
	}
	ota_relay.tracker[tracker_id].participating = false;

	uint8_t new_count = 0;
	for (uint8_t i = 0; i < ota_relay.num_targets; i++) {
		if (ota_relay.target_ids[i] != tracker_id) {
			ota_relay.target_ids[new_count++] = ota_relay.target_ids[i];
		}
	}
	ota_relay.num_targets = new_count;
	if (new_count == 0) {
		ota_relay.ring_head = 0;
		ota_relay.active = false;
	}
	return new_count == 0;
}

void esb_ota_forget_tracker(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}
	unsigned int irq_key = irq_lock();
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	bool ended = remove_target_locked(tracker_id);
	memset(&ota_relay.tracker[tracker_id], 0, sizeof(ota_relay.tracker[tracker_id]));
	ota_abort_valid[tracker_id] = false;
	ota_abort_expiry_ticks[tracker_id] = 0;
	k_spin_unlock(&ota_relay_lock, key);
	if (fw_info_pending.pending && fw_info_pending.tracker_id == tracker_id) {
		fw_info_pending.pending = false;
	}
	irq_unlock(irq_key);
	if (ended) {
		unsuppress_all_trackers();
	}
}

bool esb_ota_relay_abort_pending(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return false;
	}
	unsigned int irq_key = irq_lock();
	uint32_t now = k_uptime_ticks();
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	bool pending = ota_abort_pending_locked(tracker_id, now);
	k_spin_unlock(&ota_relay_lock, key);
	irq_unlock(irq_key);
	return pending;
}

static void queue_abort_locked(uint8_t tracker_id, uint32_t now)
{
	ota_abort_expiry_ticks[tracker_id] = now + OTA_ABORT_TOMBSTONE_TICKS;
	ota_abort_valid[tracker_id] = true;
}

static void cancel_target(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}
	unsigned int irq_key = irq_lock();
	uint32_t now = k_uptime_ticks();
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	queue_abort_locked(tracker_id, now);
	bool ended = remove_target_locked(tracker_id);
	k_spin_unlock(&ota_relay_lock, key);
	irq_unlock(irq_key);
	esb_send_remote_command(tracker_id, ESB_PONG_FLAG_OTA_ABORT);
	if (ended) {
		LOG_INF("OTA: Last target removed, session ended");
		unsuppress_all_trackers();
	}
}

static void cancel_all_targets(void)
{
	uint8_t targets[OTA_MAX_PARALLEL];
	unsigned int irq_key = irq_lock();
	uint32_t now = k_uptime_ticks();
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	uint8_t count = ota_relay.num_targets;
	memcpy(targets, ota_relay.target_ids, count);
	bool ended = false;
	for (uint8_t i = 0; i < count; i++) {
		queue_abort_locked(targets[i], now);
		ended = remove_target_locked(targets[i]);
	}
	reset_session_locked();
	k_spin_unlock(&ota_relay_lock, key);
	irq_unlock(irq_key);

	for (uint8_t i = 0; i < count; i++) {
		esb_send_remote_command(targets[i], ESB_PONG_FLAG_OTA_ABORT);
	}
	if (ended) {
		LOG_INF("OTA: Last target removed, session ended");
		unsuppress_all_trackers();
	}
	unsuppress_all_trackers();
}

static void set_pending_command_locked(struct ota_per_tracker *t, const uint8_t *packet,
				       uint8_t length, uint32_t now)
{
	memcpy(t->pending_cmd_data, packet, length);
	t->pending_cmd_len = length;
	t->pending_cmd_set_ticks = now;
	t->pending_cmd_send_count = 0;
	t->pending_cmd_type = packet[0];
}

/* ── HID OUT Report Processing (Thread Context) ─────────────────── */

void esb_ota_relay_process_hid(const uint8_t *data, size_t len)
{
	if (len < 2) {
		return;
	}

	uint8_t type = data[0];

	switch (type) {
	case HID_OTA_QUERY_INFO: {
		uint8_t tracker_id = data[1];
		if (tracker_id >= MAX_TRACKERS) {
			LOG_ERR("OTA: Invalid tracker ID %u", tracker_id);
			break;
		}
		LOG_INF("OTA: Querying firmware info for tracker %u", tracker_id);
		esb_send_remote_command(tracker_id, ESB_PONG_FLAG_OTA_QUERY_INFO);
		break;
	}

	case HID_OTA_BEGIN: {
		uint8_t tracker_id = data[1];
		if (tracker_id >= MAX_TRACKERS) {
			LOG_ERR("OTA: Invalid tracker ID %u", tracker_id);
			break;
		}
		k_spinlock_key_t admission_key = k_spin_lock(&ota_relay_lock);
		bool full = !ota_relay.tracker[tracker_id].participating &&
			    ota_relay.num_targets >= OTA_MAX_PARALLEL;
		bool needs_radio = !ota_relay.active;
		k_spin_unlock(&ota_relay_lock, admission_key);
		if (full) {
			LOG_ERR("OTA: Max parallel targets (%d) reached", OTA_MAX_PARALLEL);
			break;
		}
		bool acquired_radio = false;
		if (needs_radio) {
			if (esb_radio_operation_begin()) {
				LOG_WRN("OTA BEGIN deferred: radio operation busy");
				break;
			}
			acquired_radio = true;
			esb_auto_loss_invalidate();
		}

		uint32_t image_size = sys_get_le32(&data[2]);
		uint32_t image_crc32 = sys_get_le32(&data[6]);
		uint16_t total_packets = sys_get_be16(&data[10]);
		uint8_t proto_ver = data[12];
		const char *board_target = (const char *)&data[13];

		uint8_t begin_pkt[OTA_BEGIN_PACKET_SIZE] = {0};
		begin_pkt[0] = ESB_OTA_BEGIN_TYPE;
		begin_pkt[1] = tracker_id;
		sys_put_le32(image_size, &begin_pkt[2]);
		sys_put_le32(image_crc32, &begin_pkt[6]);
		sys_put_be16(total_packets, &begin_pkt[10]);
		begin_pkt[12] = proto_ver;
		strncpy((char *)&begin_pkt[13], board_target, OTA_BOARD_TARGET_MAX - 1);
		if (len >= 63) {
			begin_pkt[61] = data[61];
			begin_pkt[62] = data[62];
		}
		begin_pkt[63] = crc8_ccitt(0, begin_pkt, 63);

		for (;;) {
			unsigned int irq_key = irq_lock();
			uint32_t now = k_uptime_ticks();
			k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
			struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
			if (!t->participating && ota_relay.num_targets >= OTA_MAX_PARALLEL) {
				k_spin_unlock(&ota_relay_lock, key);
				irq_unlock(irq_key);
				if (acquired_radio) {
					esb_radio_operation_end();
				}
				LOG_ERR("OTA: Max parallel targets (%d) reached", OTA_MAX_PARALLEL);
				break;
			}
			if (!ota_relay.active && !acquired_radio) {
				k_spin_unlock(&ota_relay_lock, key);
				irq_unlock(irq_key);
				if (esb_radio_operation_begin()) {
					LOG_WRN("OTA BEGIN deferred: radio operation busy");
					break;
				}
				acquired_radio = true;
				esb_auto_loss_invalidate();
				continue;
			}

			bool first = !ota_relay.active;
			bool already = t->participating;
			if (first) {
				reset_session_locked();
				ota_relay.image_size = image_size;
				ota_relay.total_packets = total_packets;
			}
			uint8_t target_number = ota_relay.num_targets + 1;
			if (!t->participating) {
				ota_relay.target_ids[ota_relay.num_targets++] = tracker_id;
			}
			t->participating = true;
			t->status = OTA_STATUS_IDLE;
			t->next_seq = 0;
			t->last_hid_forward_time = 0;
			set_pending_command_locked(t, begin_pkt, sizeof(begin_pkt), now);
			ota_relay.active = true;
			ota_abort_expiry_ticks[tracker_id] = 0;
			ota_abort_valid[tracker_id] = false;
			uint8_t num_targets = ota_relay.num_targets;
			k_spin_unlock(&ota_relay_lock, key);

			/* Keep tombstone and ordinary abort-command clearing IRQ-atomic. */
			esb_clear_remote_ota_abort(tracker_id);
			irq_unlock(irq_key);

			if (already) {
				LOG_WRN("OTA: Tracker %u already in session, resetting state", tracker_id);
			}
			LOG_INF("OTA: BEGIN for tracker %u [target %u/%u], size=%u, crc=0x%08X, board=%s",
				tracker_id, target_number, OTA_MAX_PARALLEL,
				image_size, image_crc32, board_target);
			if (first) {
				suppress_non_ota_trackers();
			}
			if (acquired_radio) {
				esb_radio_operation_end();
			}
			LOG_INF("OTA: Session active with %u target(s)", num_targets);
			break;
		}
		break;
	}

	case HID_OTA_DATA: {
		k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
		if (!ota_relay.active) {
			k_spin_unlock(&ota_relay_lock, key);
			LOG_WRN("OTA: Data received but no active session");
			break;
		}

		if (len < 5) {
			k_spin_unlock(&ota_relay_lock, key);
			LOG_ERR("OTA: HID_OTA_DATA too short (%zu)", len);
			break;
		}

		if (ring_count_locked() >= OTA_TX_RING_SIZE) {
			k_spin_unlock(&ota_relay_lock, key);
			LOG_WRN("OTA: Ring buffer full, dropping packet seq=%u",
				sys_get_be16(&data[2]));
			break;
		}

		/* Build ESB OTA data packet into seq-indexed ring buffer.
		 * ring[seq & MASK] stores the DATA packet for that seq.
		 * tracker_id in data[1] is overwritten per-tracker in fill_ack. */
		uint16_t seq = sys_get_be16(&data[2]);
		uint32_t idx = seq & OTA_TX_RING_MASK;
		struct ota_tx_entry *entry = &ota_relay.ring[idx];

		entry->data[0] = ESB_OTA_DATA_TYPE;
		entry->data[1] = 0; /* Will be overwritten per-tracker in fill_ack */
		/* Sequence number: data[2-3] */
		entry->data[2] = data[2];
		entry->data[3] = data[3];
		/* Firmware data: data[4..] → entry->data[4..] */
		size_t payload_len = (len >= OTA_DATA_HEADER_SIZE) ? (len - OTA_DATA_HEADER_SIZE) : 0;
		if (payload_len > OTA_DATA_MAX_PAYLOAD) {
			payload_len = OTA_DATA_MAX_PAYLOAD;
		}
		memcpy(&entry->data[4], &data[4], payload_len);
		entry->length = OTA_DATA_HEADER_SIZE + payload_len;

		/* Advance ring_head to track the highest seq written + 1 */
		if ((uint32_t)(seq + 1) > ota_relay.ring_head) {
			ota_relay.ring_head = seq + 1;
		}
		k_spin_unlock(&ota_relay_lock, key);

		break;
	}

	case HID_OTA_VERIFY:
	case HID_OTA_ACTIVATE: {
		bool verify = type == HID_OTA_VERIFY;
		uint8_t tracker_id = data[1];
		uint8_t pkt[4] = {
			verify ? ESB_OTA_VERIFY_TYPE : ESB_OTA_ACTIVATE_TYPE,
			tracker_id, 0, 0
		};
		unsigned int irq_key = irq_lock();
		uint32_t now = k_uptime_ticks();
		k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
		bool active = ota_relay.active;
		bool target = tracker_id < MAX_TRACKERS &&
			      ota_relay.tracker[tracker_id].participating;
		if (active && target) {
			set_pending_command_locked(&ota_relay.tracker[tracker_id], pkt, sizeof(pkt), now);
		}
		k_spin_unlock(&ota_relay_lock, key);
		irq_unlock(irq_key);
		if (!active) {
			LOG_WRN("OTA: %s requested but no active session", verify ? "Verify" : "Activate");
		} else if (!target) {
			LOG_ERR("OTA: %s for non-participating tracker %u",
				verify ? "Verify" : "Activate", tracker_id);
		} else {
			LOG_INF("OTA: %s command queued for tracker %u",
				verify ? "Verify" : "Activate", tracker_id);
		}
		break;
	}

	case HID_OTA_ABORT: {
		uint8_t tracker_id = data[1];
		LOG_WRN("OTA: Abort requested for tracker %u", tracker_id);
		if (tracker_id == 0xFF) {
			cancel_all_targets();
		} else if (tracker_id < MAX_TRACKERS) {
			cancel_target(tracker_id);
		}
		break;
	}

	default:
		break;
	}
}

/* ── ESB ACK Payload Fill (Radio ISR Context) ────────────────────── */

void esb_ota_relay_fill_ack(uint8_t tracker_id, uint32_t pipe_id,
				    struct esb_payload *ack_payload, bool *has_ack,
				    const uint8_t *rx_data, uint8_t rx_len)
{
	*has_ack = false;

	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	unsigned int irq_key = irq_lock();
	uint32_t now = k_uptime_ticks();
	k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
	/* Tombstones live outside session state and therefore survive removal. */
	if (ota_abort_pending_locked(tracker_id, now)) {
		k_spin_unlock(&ota_relay_lock, key);
		irq_unlock(irq_key);
		ack_payload->pipe = pipe_id;
		ack_payload->length = ESB_PONG_LEN;
		ack_payload->noack = false;
		memset(ack_payload->data, 0, ESB_PONG_LEN);
		ack_payload->data[0] = ESB_PONG_TYPE;
		ack_payload->data[1] = tracker_id;
		/* A legacy staged-OTA tracker may also parse this as a clock PONG. */
		sys_put_be32((uint32_t)k_uptime_ticks(), &ack_payload->data[3]);
		ack_payload->data[7] = ESB_PONG_FLAG_OTA_ABORT;
		ack_payload->data[ESB_PONG_LEN - 1] = crc8_ccitt(0x07, ack_payload->data, ESB_PONG_LEN - 1);
		*has_ack = true;
		return;
	}

	if (!ota_relay.active) {
		goto out;
	}

	struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
	if (!t->participating) {
		goto out;
	}

	/* Priority 1: pending command (BEGIN, VERIFY, ACTIVATE)
	 * Commands are retried in every ACK until the tracker confirms
	 * receipt (status change) or a 10-second timeout expires.
	 * This prevents silent loss when a single ACK is missed.
	 *
	 * Exception: VERIFY/ACTIVATE must NOT block DATA delivery.
		 * Older RAM engines only process DATA packets (type 0x20) while
		 * receiving. If VERIFY replaces required DATA, the tracker stalls.
	 * Defer VERIFY/ACTIVATE when the ring has data for this tracker. */
	if (t->pending_cmd_type != 0) {
		/* Timeout: stop retrying after 10 seconds */
		uint32_t elapsed = now - t->pending_cmd_set_ticks;
		if (elapsed > k_ms_to_ticks_floor32(10000)) {
			t->pending_cmd_type = 0;
			/* Can't LOG from ISR; timeout is diagnosed by send_count */
			goto out;
		}

		/* For VERIFY/ACTIVATE: check if ring has DATA the tracker needs.
		 * If so, skip the command and fall through to send DATA instead. */
		bool defer_cmd = false;
		if (t->pending_cmd_type != ESB_OTA_BEGIN_TYPE &&
		    rx_data != NULL && rx_len >= 5 &&
		    rx_data[0] == ESB_OTA_STATUS_TYPE) {
			uint16_t ns = sys_get_be16(&rx_data[3]);
			if ((uint32_t)ns < ota_relay.ring_head) {
				uint32_t idx = ns & OTA_TX_RING_MASK;
				if (sys_get_be16(&ota_relay.ring[idx].data[2]) == ns) {
					defer_cmd = true;
				}
			}
		}

		if (!defer_cmd) {
			ack_payload->pipe = pipe_id;
			ack_payload->length = t->pending_cmd_len;
			ack_payload->noack = false;
			memcpy(ack_payload->data, t->pending_cmd_data, t->pending_cmd_len);
			t->pending_cmd_send_count++;
			/* Don't clear pending_cmd_type — keep retrying until
			 * tracker confirms via OTA_STATUS or timeout expires. */
			*has_ack = true;
			goto out;
		}
		/* defer_cmd == true: fall through to DATA section */
	}

	/* Priority 2: data indexed by tracker's next_seq from STATUS.
	 *
	 * The tracker reports which seq it needs in every STATUS packet.
	 * We look up that seq in the ring buffer and send it.
	 * If the previous ACK was lost, the tracker re-sends the same
	 * next_seq, and we automatically retransmit the same packet.
	 *
	 * Only send data when rx_data is a valid OTA STATUS packet;
	 * for normal PONG path (rx_data is NULL or non-OTA), skip. */
	if (rx_data == NULL || rx_len < 5 ||
	    rx_data[0] != ESB_OTA_STATUS_TYPE) {
		goto out;
	}

	uint16_t next_seq = sys_get_be16(&rx_data[3]);
	t->next_seq = next_seq;

	/* Check if this seq is available in the ring buffer */
	if ((uint32_t)next_seq >= ota_relay.ring_head) {
		goto out; /* PC hasn't sent this seq yet */
	}

	uint32_t idx = next_seq & OTA_TX_RING_MASK;
	struct ota_tx_entry *entry = &ota_relay.ring[idx];

	/* Validate the entry hasn't been overwritten by ring wraparound */
	uint16_t entry_seq = sys_get_be16(&entry->data[2]);
	if (entry_seq != next_seq) {
		goto out; /* Stale entry — PC needs to re-send */
	}

	ack_payload->pipe = pipe_id;
	ack_payload->length = entry->length;
	ack_payload->noack = false;
	memcpy(ack_payload->data, entry->data, entry->length);
	/* Patch tracker_id for this specific tracker */
	ack_payload->data[1] = tracker_id;
	*has_ack = true;

out:
	k_spin_unlock(&ota_relay_lock, key);
	irq_unlock(irq_key);
}

/* ── Tracker → PC Forwarding (Event Handler Context) ─────────────── */

void esb_ota_relay_process_tracker_packet(const uint8_t *data, size_t len)
{
	if (len < 2) {
		return;
	}

	uint8_t type = data[0];
	uint8_t tracker_id = data[1];

	switch (type) {
	case ESB_OTA_STATUS_TYPE: {
		if (len < 3 || tracker_id >= MAX_TRACKERS) {
			break;
		}

		uint8_t status = data[2];

		uint16_t next_seq = (len >= 5) ? sys_get_be16(&data[3]) : 0;
		uint32_t bytes_written = (len >= 9) ? sys_get_le32(&data[5]) : 0;

		/* Forward state changes and terminal statuses, or every 30 ms. */
		bool is_terminal = (status == OTA_STATUS_COMPLETE ||
				    status == OTA_STATUS_ERROR ||
				    status == OTA_STATUS_VERIFY_FAIL ||
				    status == OTA_STATUS_TIMEOUT ||
				    status == OTA_STATUS_BOARD_MISMATCH ||
				    status == OTA_STATUS_SIZE_ERROR ||
				    status == OTA_STATUS_FLASH_ERROR ||
				    status == OTA_STATUS_SEQ_ERROR);
		unsigned int irq_key = irq_lock();
		int64_t now = k_uptime_get();
		uint8_t confirmed_type = 0;
		uint32_t confirmed_count = 0;
		uint32_t rc = 0;
		uint8_t num_targets = 0;
		k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
		struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
		uint8_t prev_status = t->status;
		t->status = status;

		bool confirmed = false;
		switch (t->pending_cmd_type) {
		case ESB_OTA_BEGIN_TYPE:
			confirmed = status != OTA_STATUS_IDLE;
			break;
		case ESB_OTA_VERIFY_TYPE:
			confirmed = status == OTA_STATUS_VERIFY_OK ||
				    status == OTA_STATUS_VERIFY_FAIL;
			break;
		case ESB_OTA_ACTIVATE_TYPE:
			confirmed = status == OTA_STATUS_ACTIVATING ||
				    status == OTA_STATUS_COMPLETE;
			break;
		}
		if (confirmed) {
			confirmed_type = t->pending_cmd_type;
			confirmed_count = t->pending_cmd_send_count;
			t->pending_cmd_type = 0;
		}

		bool forward = status != prev_status || is_terminal ||
			       now - t->last_hid_forward_time >= 30;
		if (forward) {
			t->last_hid_forward_time = now;
			rc = ring_count_locked();
			num_targets = ota_relay.num_targets;
		}
		bool ended = is_terminal && remove_target_locked(tracker_id);
		k_spin_unlock(&ota_relay_lock, key);
		irq_unlock(irq_key);

		if (confirmed_type != 0) {
			LOG_INF("OTA: Cmd 0x%02X confirmed by tracker %u (sent %u times)",
				confirmed_type, tracker_id, confirmed_count);
		}
		if (forward) {
			if (status == OTA_STATUS_VERIFY_FAIL) {
				LOG_WRN("OTA VERIFY_FAIL: tracker=%u", tracker_id);
			}
			/* Throttled path runs from EVENT IRQ — keep UART off hot path. */
			LOG_DBG("OTA Status: tracker=%u status=0x%02X seq=%u bytes=%u ring=%u",
				tracker_id, status, next_seq, bytes_written, rc);

			/* Forward to PC via HID IN report */
			uint8_t hid_report[16] = {0};
			hid_report[0] = HID_OTA_STATUS;
			hid_report[1] = tracker_id;
			hid_report[2] = status;
			hid_report[3] = (next_seq >> 8) & 0xFF;
			hid_report[4] = next_seq & 0xFF;
			hid_report[5] = (bytes_written) & 0xFF;
			hid_report[6] = (bytes_written >> 8) & 0xFF;
			hid_report[7] = (bytes_written >> 16) & 0xFF;
			hid_report[8] = (bytes_written >> 24) & 0xFF;
			hid_report[9] = (rc > 255) ? 255 : (uint8_t)rc;
			hid_report[10] = num_targets;
			hid_write_packet_n(hid_report, 0);
		}

		/* Handle terminal status: remove this tracker from session */
		if (is_terminal) {
			LOG_INF("OTA: Tracker %u session ended (status=0x%02X)", tracker_id, status);
			if (ended) {
				LOG_INF("OTA: Last target removed, session ended");
				unsuppress_all_trackers();
			}
		}
		break;
	}

	case ESB_OTA_FW_INFO_TYPE: {
		LOG_INF("OTA: Received firmware info from tracker %u", tracker_id);

		/* Defer to system work queue so we can sleep between chunks.
		 * This avoids HID FIFO overflow when called from ESB ISR. */
		memcpy(fw_info_pending.data, data, MIN(len, 66));
		fw_info_pending.len = MIN(len, 66);
		fw_info_pending.tracker_id = tracker_id;
		fw_info_pending.pending = true;
		k_work_submit(&fw_info_send_work);
		break;
	}

	default:
		LOG_WRN("OTA: Unknown tracker packet type 0x%02X", type);
		break;
	}
}

/* ── Console Command Handler ─────────────────────────────────────── */

void esb_ota_relay_console_cmd(uint8_t tracker_id, const char *cmd)
{
	if (strcmp(cmd, "info") == 0 || strcmp(cmd, "query") == 0) {
		if (tracker_id < MAX_TRACKERS) {
			LOG_INF("OTA: Requesting firmware info from tracker %u", tracker_id);
			esb_send_remote_command(tracker_id, ESB_PONG_FLAG_OTA_QUERY_INFO);
		}
	} else if (strcmp(cmd, "abort") == 0 || strcmp(cmd, "cancel") == 0) {
		if (tracker_id == 0xFF) {
			LOG_WRN("OTA: Aborting all targets");
			cancel_all_targets();
		} else if (tracker_id < MAX_TRACKERS) {
			LOG_WRN("OTA: Sending abort to tracker %u", tracker_id);
			cancel_target(tracker_id);
		}
	} else if (strcmp(cmd, "status") == 0) {
		struct {
			uint8_t id;
			uint8_t status;
			uint16_t next_seq;
			uint32_t pending;
		} rows[OTA_MAX_PARALLEL];
		uint8_t count = 0;
		uint32_t rc = 0;
		k_spinlock_key_t key = k_spin_lock(&ota_relay_lock);
		bool active = ota_relay.active;
		if (active) {
			count = ota_relay.num_targets;
			rc = ring_count_locked();
			for (uint8_t i = 0; i < count; i++) {
				uint8_t id = ota_relay.target_ids[i];
				struct ota_per_tracker *t = &ota_relay.tracker[id];
				rows[i].id = id;
				rows[i].status = t->status;
				rows[i].next_seq = t->next_seq;
				rows[i].pending = ota_relay.ring_head - (uint32_t)t->next_seq;
			}
		}
		k_spin_unlock(&ota_relay_lock, key);
		if (active) {
			printk("OTA active: %u target(s), ring=%u/%u\n",
			       count, rc, OTA_TX_RING_SIZE);
			for (uint8_t i = 0; i < count; i++) {
				printk("  tracker %u: status=0x%02X next_seq=%u pending=%u\n",
				       rows[i].id, rows[i].status, rows[i].next_seq, rows[i].pending);
			}
		} else {
			printk("OTA: No active session\n");
		}
	} else {
		printk("OTA commands: info, abort, status\n");
	}
}
