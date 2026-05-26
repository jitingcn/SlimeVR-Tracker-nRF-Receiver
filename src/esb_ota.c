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
	uint8_t data[48];
	uint8_t length;
};

/* Per-tracker OTA state */
struct ota_per_tracker {
	volatile bool participating;         /* Is this tracker in the OTA session? */
	volatile uint8_t status;             /* Last status from this tracker */
	volatile uint16_t next_seq;          /* Last confirmed next_seq from STATUS */
	volatile uint8_t pending_cmd_type;   /* 0 = none */
	uint8_t pending_cmd_data[48];
	volatile uint8_t pending_cmd_len;
	volatile uint32_t pending_cmd_set_ticks;  /* k_uptime_ticks() when cmd was queued */
	volatile uint32_t pending_cmd_send_count; /* Times sent in ACK (for diagnostics) */
	int64_t last_hid_forward_time;       /* Throttle per-tracker HID status */
};

static struct {
	volatile bool active;
	uint8_t num_targets;
	uint8_t target_ids[OTA_MAX_PARALLEL]; /* List of participating tracker IDs */

	/* Session parameters (set during first BEGIN) */
	volatile uint32_t image_size;
	volatile uint16_t total_packets;

	/* Shared seq-indexed ring buffer (same data for all targets) */
	struct ota_tx_entry ring[OTA_TX_RING_SIZE];
	volatile uint32_t ring_head;          /* Next seq to be written (by HID thread) */

	/* Per-tracker state (indexed by tracker_id, not target index) */
	struct ota_per_tracker tracker[MAX_TRACKERS];
} ota_relay = {
	.active = false,
	.num_targets = 0,
	.ring_head = 0,
};

/* ── FW_INFO deferred send (ISR → thread context) ───────────────── */

static struct {
	uint8_t data[48];
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

	for (int chunk = 0; chunk < 4; chunk++) {
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
	return ota_relay.active;
}

bool esb_ota_relay_is_target(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return false;
	}
	return ota_relay.active && ota_relay.tracker[tracker_id].participating;
}

uint8_t esb_ota_relay_get_num_targets(void)
{
	return ota_relay.num_targets;
}

/* ── Ring Buffer Helpers ─────────────────────────────────────────── */

/*
 * Compute the minimum next_seq across all participating trackers.
 * Data below this seq is fully consumed by all trackers.
 * The producer cannot write beyond min_seq + RING_SIZE.
 */
static uint32_t ring_seq_min(void)
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

static uint32_t ring_count(void)
{
	return ota_relay.ring_head - ring_seq_min();
}

static bool ring_full(void)
{
	return ring_count() >= OTA_TX_RING_SIZE;
}

/* ── Tracker Suppression ─────────────────────────────────────────── */

/*
 * Suppress non-OTA trackers to free radio bandwidth during OTA.
 * Suppressed trackers reduce their poll rate to ~1 Hz.
 */
static void suppress_non_ota_trackers(void)
{
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (!ota_relay.tracker[i].participating) {
			esb_send_remote_command(i, ESB_PONG_FLAG_OTA_SUPPRESS);
		}
	}
}

static void unsuppress_all_trackers(void)
{
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		esb_send_remote_command(i, ESB_PONG_FLAG_OTA_UNSUPPRESS);
	}
}

/* ── Session Management ──────────────────────────────────────────── */

static void reset_session(void)
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

static void remove_target(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}
	if (!ota_relay.tracker[tracker_id].participating) {
		return;  /* Already removed */
	}
	ota_relay.tracker[tracker_id].participating = false;

	/* Rebuild target_ids array */
	uint8_t new_count = 0;
	for (uint8_t i = 0; i < ota_relay.num_targets; i++) {
		if (ota_relay.target_ids[i] != tracker_id) {
			ota_relay.target_ids[new_count++] = ota_relay.target_ids[i];
		}
	}
	ota_relay.num_targets = new_count;

	if (new_count == 0) {
		LOG_INF("OTA: Last target removed, session ended");
		ota_relay.active = false;
		unsuppress_all_trackers();
	}
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

		/* If tracker is already in session, reset its state in-place
		 * (handles tracker reboot during OTA or PC retry) */
		if (ota_relay.tracker[tracker_id].participating) {
			LOG_WRN("OTA: Tracker %u already in session, resetting state", tracker_id);
			struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
			t->next_seq = 0;  /* Tracker will re-request from seq 0 */
			t->pending_cmd_type = 0;
			t->pending_cmd_send_count = 0;
			t->last_hid_forward_time = 0;
			/* Fall through to re-setup BEGIN command below */
		}

		/* Check parallel limit */
		if (ota_relay.num_targets >= OTA_MAX_PARALLEL) {
			LOG_ERR("OTA: Max parallel targets (%d) reached", OTA_MAX_PARALLEL);
			break;
		}

		uint32_t image_size = sys_get_le32(&data[2]);
		uint32_t image_crc32 = sys_get_le32(&data[6]);
		uint16_t total_packets = sys_get_be16(&data[10]);
		uint8_t proto_ver = data[12];
		const char *board_target = (const char *)&data[13];

		/* If this is the first target, initialize session */
		if (!ota_relay.active) {
			reset_session();
			ota_relay.image_size = image_size;
			ota_relay.total_packets = total_packets;
		}

		LOG_INF("OTA: BEGIN for tracker %u [target %u/%u], size=%u, crc=0x%08X, board=%s",
			tracker_id, ota_relay.num_targets + 1, OTA_MAX_PARALLEL,
			image_size, image_crc32, board_target);

		/* Initialize per-tracker state (BEFORE suppressing so this
		 * tracker is marked as participating and won't be suppressed) */
		struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
		t->participating = true;
		t->status = OTA_STATUS_IDLE;
		t->next_seq = 0; /* Tracker starts requesting from seq 0 */
		t->pending_cmd_type = 0;
		t->pending_cmd_set_ticks = 0;
		t->pending_cmd_send_count = 0;
		t->last_hid_forward_time = 0;

		/* Add to target list (skip if already present from reset path) */
		if (!t->participating) {
			ota_relay.target_ids[ota_relay.num_targets++] = tracker_id;
		}

		/* Suppress non-OTA trackers (done after marking this tracker
		 * as participating to avoid suppressing the OTA target itself) */
		if (!ota_relay.active) {
			ota_relay.active = true;
			suppress_non_ota_trackers();
		}

		/* Build OTA_BEGIN ESB packet and queue as per-tracker pending cmd */
		uint8_t begin_pkt[OTA_BEGIN_PACKET_SIZE];
		memset(begin_pkt, 0, sizeof(begin_pkt));
		begin_pkt[0] = ESB_OTA_BEGIN_TYPE;
		begin_pkt[1] = tracker_id;
		sys_put_le32(image_size, &begin_pkt[2]);
		sys_put_le32(image_crc32, &begin_pkt[6]);
		sys_put_be16(total_packets, &begin_pkt[10]);
		begin_pkt[12] = proto_ver;
		strncpy((char *)&begin_pkt[13], board_target, OTA_BOARD_TARGET_MAX - 1);
		/* Forward flash base address (bytes 45-46 from HID report) */
		if (len >= 47) {
			begin_pkt[45] = data[45];
			begin_pkt[46] = data[46];
		}

		/* CRC-8 */
		uint8_t crc = 0;
		for (int i = 0; i < 47; i++) {
			crc ^= begin_pkt[i];
			for (int j = 0; j < 8; j++) {
				crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
			}
		}
		begin_pkt[47] = crc;

		memcpy(t->pending_cmd_data, begin_pkt, OTA_BEGIN_PACKET_SIZE);
		t->pending_cmd_len = OTA_BEGIN_PACKET_SIZE;
		t->pending_cmd_set_ticks = k_uptime_ticks();
		t->pending_cmd_send_count = 0;
		__DMB();
		t->pending_cmd_type = ESB_OTA_BEGIN_TYPE;

		LOG_INF("OTA: Session active with %u target(s)", ota_relay.num_targets);
		break;
	}

	case HID_OTA_DATA: {
		if (!ota_relay.active) {
			LOG_WRN("OTA: Data received but no active session");
			break;
		}

		if (len < 5) {
			LOG_ERR("OTA: HID_OTA_DATA too short (%zu)", len);
			break;
		}

		if (ring_full()) {
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
		/* Firmware data: data[4..47] → entry->data[4..47] */
		size_t payload_len = (len > 48) ? 44 : (len - 4);
		if (payload_len > OTA_DATA_MAX_PAYLOAD) {
			payload_len = OTA_DATA_MAX_PAYLOAD;
		}
		memcpy(&entry->data[4], &data[4], payload_len);
		entry->length = OTA_DATA_HEADER_SIZE + payload_len;

		__DMB();
		/* Advance ring_head to track the highest seq written + 1 */
		if ((uint32_t)(seq + 1) > ota_relay.ring_head) {
			ota_relay.ring_head = seq + 1;
		}

		break;
	}

	case HID_OTA_VERIFY: {
		if (!ota_relay.active) {
			LOG_WRN("OTA: Verify requested but no active session");
			break;
		}

		uint8_t tracker_id = data[1];
		if (tracker_id >= MAX_TRACKERS ||
		    !ota_relay.tracker[tracker_id].participating) {
			LOG_ERR("OTA: Verify for non-participating tracker %u", tracker_id);
			break;
		}

		LOG_INF("OTA: Verify command queued for tracker %u", tracker_id);
		struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
		uint8_t pkt[4] = {ESB_OTA_VERIFY_TYPE, tracker_id, 0, 0};
		memcpy(t->pending_cmd_data, pkt, 4);
		t->pending_cmd_len = 4;
		t->pending_cmd_set_ticks = k_uptime_ticks();
		t->pending_cmd_send_count = 0;
		__DMB();
		t->pending_cmd_type = ESB_OTA_VERIFY_TYPE;
		break;
	}

	case HID_OTA_ACTIVATE: {
		if (!ota_relay.active) {
			LOG_WRN("OTA: Activate requested but no active session");
			break;
		}

		uint8_t tracker_id = data[1];
		if (tracker_id >= MAX_TRACKERS ||
		    !ota_relay.tracker[tracker_id].participating) {
			LOG_ERR("OTA: Activate for non-participating tracker %u", tracker_id);
			break;
		}

		LOG_INF("OTA: Activate command queued for tracker %u", tracker_id);
		struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
		uint8_t pkt[4] = {ESB_OTA_ACTIVATE_TYPE, tracker_id, 0, 0};
		memcpy(t->pending_cmd_data, pkt, 4);
		t->pending_cmd_len = 4;
		t->pending_cmd_set_ticks = k_uptime_ticks();
		t->pending_cmd_send_count = 0;
		__DMB();
		t->pending_cmd_type = ESB_OTA_ACTIVATE_TYPE;
		break;
	}

	case HID_OTA_ABORT: {
		uint8_t tracker_id = data[1];
		LOG_WRN("OTA: Abort requested for tracker %u", tracker_id);

		if (tracker_id == 0xFF) {
			/* Abort all targets */
			for (uint8_t i = 0; i < ota_relay.num_targets; i++) {
				esb_send_remote_command(ota_relay.target_ids[i],
							ESB_PONG_FLAG_OTA_ABORT);
			}
			reset_session();
			unsuppress_all_trackers();
		} else if (tracker_id < MAX_TRACKERS &&
			   ota_relay.tracker[tracker_id].participating) {
			esb_send_remote_command(tracker_id, ESB_PONG_FLAG_OTA_ABORT);
			remove_target(tracker_id);
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

	if (!ota_relay.active || tracker_id >= MAX_TRACKERS) {
		return;
	}

	struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
	if (!t->participating) {
		return;
	}

	/* Priority 1: pending command (BEGIN, VERIFY, ACTIVATE)
	 * Commands are retried in every ACK until the tracker confirms
	 * receipt (status change) or a 10-second timeout expires.
	 * This prevents silent loss when a single ACK is missed. */
	if (t->pending_cmd_type != 0) {
		/* Timeout: stop retrying after 10 seconds */
		uint32_t elapsed = (uint32_t)k_uptime_ticks() - t->pending_cmd_set_ticks;
		if (elapsed > k_ms_to_ticks_floor32(10000)) {
			t->pending_cmd_type = 0;
			/* Can't LOG from ISR; timeout is diagnosed by send_count */
			return;
		}

		ack_payload->pipe = pipe_id;
		ack_payload->length = t->pending_cmd_len;
		ack_payload->noack = false;
		memcpy(ack_payload->data, t->pending_cmd_data, t->pending_cmd_len);
		t->pending_cmd_send_count++;
		/* Don't clear pending_cmd_type — keep retrying until
		 * tracker confirms via OTA_STATUS or timeout expires. */
		*has_ack = true;
		return;
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
		return;
	}

	uint16_t next_seq = sys_get_be16(&rx_data[3]);
	t->next_seq = next_seq;

	/* Check if this seq is available in the ring buffer */
	if ((uint32_t)next_seq >= ota_relay.ring_head) {
		return; /* PC hasn't sent this seq yet */
	}

	uint32_t idx = next_seq & OTA_TX_RING_MASK;
	struct ota_tx_entry *entry = &ota_relay.ring[idx];

	/* Validate the entry hasn't been overwritten by ring wraparound */
	uint16_t entry_seq = sys_get_be16(&entry->data[2]);
	if (entry_seq != next_seq) {
		return; /* Stale entry — PC needs to re-send */
	}

	ack_payload->pipe = pipe_id;
	ack_payload->length = entry->length;
	ack_payload->noack = false;
	memcpy(ack_payload->data, entry->data, entry->length);
	/* Patch tracker_id for this specific tracker */
	ack_payload->data[1] = tracker_id;
	*has_ack = true;
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

		struct ota_per_tracker *t = &ota_relay.tracker[tracker_id];
		uint8_t status = data[2];
		uint8_t prev_status = t->status;
		t->status = status;

		/* Clear pending command once tracker confirms receipt.
		 * The tracker echoes its state in OTA_STATUS; a state
		 * transition matching the pending command means it arrived. */
		if (t->pending_cmd_type != 0) {
			bool confirmed = false;
			switch (t->pending_cmd_type) {
			case ESB_OTA_BEGIN_TYPE:
				confirmed = (status != OTA_STATUS_IDLE);
				break;
			case ESB_OTA_VERIFY_TYPE:
				confirmed = (status == OTA_STATUS_VERIFY_OK ||
					     status == OTA_STATUS_VERIFY_FAIL);
				break;
			case ESB_OTA_ACTIVATE_TYPE:
				confirmed = (status == OTA_STATUS_ACTIVATING ||
					     status == OTA_STATUS_COMPLETE);
				break;
			}
			if (confirmed) {
				LOG_INF("OTA: Cmd 0x%02X confirmed by tracker %u "
					"(sent %u times)",
					t->pending_cmd_type, tracker_id,
					t->pending_cmd_send_count);
				t->pending_cmd_type = 0;
			}
		}

		uint16_t next_seq = (len >= 5) ? sys_get_be16(&data[3]) : 0;
		uint32_t bytes_written = (len >= 9) ? sys_get_le32(&data[5]) : 0;

		/*
		 * Throttle HID status forwarding per tracker:
		 * - Always forward on status change (state transitions)
		 * - Always forward terminal statuses (complete/error)
		 * - Otherwise limit to ~5 Hz (every 200ms)
		 */
		bool is_terminal = (status == OTA_STATUS_COMPLETE ||
				    status == OTA_STATUS_ERROR ||
				    status == OTA_STATUS_VERIFY_FAIL ||
				    status == OTA_STATUS_TIMEOUT ||
				    status == OTA_STATUS_BOARD_MISMATCH ||
				    status == OTA_STATUS_SIZE_ERROR);
		bool status_changed = (status != prev_status);

		int64_t now = k_uptime_get();
		bool throttle_due = (now - t->last_hid_forward_time >= 200);

		if (status_changed || is_terminal || throttle_due) {
			t->last_hid_forward_time = now;

			if (status == OTA_STATUS_VERIFY_FAIL) {
				LOG_WRN("OTA VERIFY_FAIL: tracker=%u", tracker_id);
			}

			LOG_INF("OTA Status: tracker=%u status=0x%02X seq=%u bytes=%u ring=%u",
				tracker_id, status, next_seq, bytes_written, ring_count());

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
			uint32_t rc = ring_count();
			hid_report[9] = (rc > 255) ? 255 : (uint8_t)rc;
			hid_report[10] = ota_relay.num_targets;
			hid_write_packet_n(hid_report, 0);
		}

		/* Handle terminal status: remove this tracker from session */
		if (is_terminal) {
			LOG_INF("OTA: Tracker %u session ended (status=0x%02X)", tracker_id, status);
			remove_target(tracker_id);
		}
		break;
	}

	case ESB_OTA_FW_INFO_TYPE: {
		LOG_INF("OTA: Received firmware info from tracker %u", tracker_id);

		/* Defer to system work queue so we can sleep between chunks.
		 * This avoids HID FIFO overflow when called from ESB ISR. */
		memcpy(fw_info_pending.data, data, MIN(len, 48));
		fw_info_pending.len = MIN(len, 48);
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
		LOG_INF("OTA: Requesting firmware info from tracker %u", tracker_id);
		esb_send_remote_command(tracker_id, ESB_PONG_FLAG_OTA_QUERY_INFO);
	} else if (strcmp(cmd, "abort") == 0 || strcmp(cmd, "cancel") == 0) {
		if (tracker_id == 0xFF) {
			/* Abort all targets */
			LOG_WRN("OTA: Aborting all targets");
			for (uint8_t i = 0; i < ota_relay.num_targets; i++) {
				esb_send_remote_command(ota_relay.target_ids[i],
							ESB_PONG_FLAG_OTA_ABORT);
			}
			reset_session();
			unsuppress_all_trackers();
		} else {
			LOG_WRN("OTA: Sending abort to tracker %u", tracker_id);
			esb_send_remote_command(tracker_id, ESB_PONG_FLAG_OTA_ABORT);
			if (ota_relay.tracker[tracker_id].participating) {
				remove_target(tracker_id);
			}
		}
	} else if (strcmp(cmd, "status") == 0) {
		if (ota_relay.active) {
			printk("OTA active: %u target(s), ring=%u/%u\n",
			       ota_relay.num_targets, ring_count(), OTA_TX_RING_SIZE);
			for (uint8_t i = 0; i < ota_relay.num_targets; i++) {
				uint8_t id = ota_relay.target_ids[i];
				struct ota_per_tracker *t = &ota_relay.tracker[id];
				uint32_t pending = ota_relay.ring_head - (uint32_t)t->next_seq;
				printk("  tracker %u: status=0x%02X next_seq=%u pending=%u\n",
				       id, t->status, t->next_seq, pending);
			}
		} else {
			printk("OTA: No active session\n");
		}
	} else {
		printk("OTA commands: info, abort, status\n");
	}
}
