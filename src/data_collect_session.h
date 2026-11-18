/* SPDX-License-Identifier: MIT */
#ifndef SLIMENRF_DATA_COLLECT_SESSION_H
#define SLIMENRF_DATA_COLLECT_SESSION_H

/* Private implementation shared by the mutually exclusive CDC and HID backends.
 * On the receiver's single core, irq_lock() serializes thread-side transitions
 * with each other and with the ESB EVENT ISR, the sole RX producer. Keep this
 * lock around the local transition and the remote command stores, not logging
 * or USB I/O. Transport hooks only reset local queue ownership and counters.
 */
static void dc_reset_output_locked(void);
static void dc_reset_stats_locked(int64_t now);

static bool dc_active;
static uint8_t dc_target_tracker_id;
static bool dc_batch_active;
static uint32_t dc_batch_mask;

#define DC_TIMEOUT_MS 60000
static int64_t dc_last_rx_time;
static bool dc_timeout_pending;

/* Called only for accepted data, in the ESB EVENT ISR. Even if USB cannot
 * forward this packet, the tracker is still producing data.
 */
static inline void dc_note_rx(void)
{
	dc_last_rx_time = k_uptime_get();
	dc_timeout_pending = false;
}

struct dc_stopped {
	uint32_t mask;
	uint32_t sent;
	uint32_t dropped;
#ifdef CONFIG_DATA_COLLECT_HID
	uint32_t received;
#endif
	uint8_t tracker_id;
	bool batch;
};

static struct dc_stopped dc_stop_locked(bool batch)
{
	if (!(batch ? dc_batch_active : dc_active)) {
		return (struct dc_stopped){0};
	}

	struct dc_stopped stopped = {
		.mask = batch ? dc_batch_mask : BIT(dc_target_tracker_id),
		.sent = dc_frames_sent,
		.dropped = dc_frames_dropped,
#ifdef CONFIG_DATA_COLLECT_HID
		.received = dc_frames_received,
#endif
		.tracker_id = dc_target_tracker_id,
		.batch = batch,
	};
	if (batch) {
		dc_batch_active = false;
		dc_batch_mask = 0;
	} else {
		dc_active = false;
	}
	dc_timeout_pending = false;
	dc_reset_output_locked();
	if (batch) {
		for (uint8_t id = 0; id < MAX_TRACKERS; id++) {
			if (stopped.mask & BIT(id)) {
				esb_publish_remote_command_locked(id, ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF, 0);
			}
		}
	} else {
		esb_publish_remote_command_locked(stopped.tracker_id, ESB_PONG_FLAG_DATA_COLLECT_OFF, 0);
	}
	return stopped;
}

static void dc_log_stop(struct dc_stopped stopped)
{
	if (stopped.mask == 0) {
		return;
	}
#ifdef CONFIG_DATA_COLLECT_HID
	LOG_INF("%sdata collection STOPPED (received: %u, sent: %u, dropped: %u)",
		stopped.batch ? "Batch " : "", stopped.received, stopped.sent, stopped.dropped);
#else
	LOG_INF("%sdata collection STOPPED (sent: %u, dropped: %u)",
		stopped.batch ? "Batch " : "", stopped.sent, stopped.dropped);
#endif
}

static void dc_stop(bool batch)
{
	unsigned int key = irq_lock();
	struct dc_stopped stopped = dc_stop_locked(batch);
	irq_unlock(key);
	dc_log_stop(stopped);
}

void data_collect_stop(void)
{
	dc_stop(false);
}

void data_collect_batch_stop(void)
{
	dc_stop(true);
}

static void dc_start(uint8_t tracker_id, uint32_t mask, uint16_t rate_hz, bool batch)
{
	unsigned int key = irq_lock();
	struct dc_stopped stopped = dc_stop_locked(dc_batch_active);
	if (stopped.mask == 0) {
		dc_reset_output_locked();
	}
	int64_t now = k_uptime_get();
	dc_reset_stats_locked(now);
	dc_last_rx_time = now;
	dc_timeout_pending = false;
	if (batch) {
		dc_batch_mask = mask;
		dc_batch_active = mask != 0;
		for (uint8_t id = 0; id < MAX_TRACKERS; id++) {
			if (mask & BIT(id)) {
				esb_publish_remote_command_locked(id, ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON, (uint8_t)rate_hz);
			}
		}
	} else {
		dc_target_tracker_id = tracker_id;
		dc_active = true;
		esb_publish_remote_command_locked(tracker_id, ESB_PONG_FLAG_DATA_COLLECT_ON, 0);
	}
	irq_unlock(key);
	dc_log_stop(stopped);
}

void data_collect_start(uint8_t tracker_id)
{
	dc_start(tracker_id, 0, 0, false);
	LOG_INF("Data collection STARTED for tracker %u", tracker_id);
}

void data_collect_batch_start(uint32_t mask, uint16_t rate_hz)
{
	dc_start(0, mask, rate_hz, true);
	LOG_INF("Batch data collection STARTED mask=0x%08x rate=%u Hz", (unsigned int)mask, rate_hz);
}

static bool dc_expired_locked(void)
{
	return (dc_active || dc_batch_active) && k_uptime_get() - dc_last_rx_time > DC_TIMEOUT_MS;
}

static void dc_timeout_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	struct dc_stopped stopped = {0};
	unsigned int key = irq_lock();
	/* The timer is only a hint. RX and lifecycle changes revoke its claim;
	 * recheck and commit the local stop and all OFF flags in one transaction.
	 */
	if (dc_timeout_pending && dc_expired_locked()) {
		stopped = dc_stop_locked(dc_batch_active);
	}
	dc_timeout_pending = false;
	irq_unlock(key);
	if (stopped.mask == 0) {
		return;
	}

	dc_log_stop(stopped);
	if (stopped.batch) {
		LOG_WRN("Batch data collection timed out (no data for %d s), sent OFF to trackers 0x%08x",
			DC_TIMEOUT_MS / 1000, (unsigned int)stopped.mask);
	} else {
		LOG_WRN("Data collection timed out (no data for %d s), sent OFF to tracker %u",
			DC_TIMEOUT_MS / 1000, stopped.tracker_id);
	}
}
static K_WORK_DEFINE(dc_timeout_work, dc_timeout_work_handler);

static void dc_timeout_check(void)
{
	unsigned int key = irq_lock();
	bool expired = dc_expired_locked();
	if (expired) {
		dc_timeout_pending = true;
	}
	irq_unlock(key);
	if (expired) {
		k_work_submit(&dc_timeout_work);
	}
}
#endif /* SLIMENRF_DATA_COLLECT_SESSION_H */
