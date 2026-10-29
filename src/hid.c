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
#include "globals.h"
#include "hid.h"
#include "connection/esb.h"
#include "esb_ota.h"
#include "receiver_ota.h"
#include "rcv_cmd.h"
#include "rcv_hid_cmd.h"
#include "usb.h"
#include "tracker_events.h"

#include <limits.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/class/usbd_hid.h>

static struct k_work_delayable report_send;

static struct tracker_report {
	uint8_t data[16];
} __packed report = {
	.data = {0}
};

#define MAX_REPORTS (MAX_TRACKERS * 4)
BUILD_ASSERT((MAX_REPORTS & (MAX_REPORTS - 1)) == 0, "MAX_REPORTS must be power of two");
/* Keep a few slots free for CMD_ACK / OTA status under stream load. */
#define HID_FIFO_PRIORITY_RESERVE 2
#define HID_FIFO_MASK (MAX_REPORTS - 1)

/*
 * Lock-free bounded MPSC ring (ticket + per-slot seq).
 * Producers: ESB EVENT IRQ, USB/cmd threads. Consumer: send_report work.
 * Claim write_pos with CAS, publish via seq = pos+1; consumer waits seq == read+1.
 */
struct hid_fifo_slot {
	uint8_t data[16];
	/* Zero identifies ordinary records, which never undergo lease filtering. */
	uint32_t subscription_generation;
	atomic_t seq;
};

static struct hid_fifo_slot hid_fifo_slots[MAX_REPORTS];
static atomic_t hid_fifo_write_pos = ATOMIC_INIT(0);
static atomic_t hid_fifo_read_pos = ATOMIC_INIT(0);

static void hid_fifo_init(void)
{
	for (uint32_t i = 0; i < MAX_REPORTS; i++) {
		atomic_set(&hid_fifo_slots[i].seq, (atomic_val_t)i);
	}
	atomic_set(&hid_fifo_write_pos, 0);
	atomic_set(&hid_fifo_read_pos, 0);
}

static bool hid_fifo_is_priority(uint8_t type)
{
	return type == RCV_HID_TYPE_CMD_ACK || (type >= 0xF0 && type <= 0xF7);
}

static bool hid_fifo_try_push(const uint8_t data[16], bool priority, uint32_t generation)
{
	uint32_t pos;

	for (;;) {
		pos = (uint32_t)atomic_get(&hid_fifo_write_pos);
		uint32_t read = (uint32_t)atomic_get(&hid_fifo_read_pos);
		uint32_t count = pos - read;

		if (count >= MAX_REPORTS) {
			return false;
		}
		uint32_t free_slots = MAX_REPORTS - count;
		if (!priority && free_slots <= HID_FIFO_PRIORITY_RESERVE) {
			return false;
		}
		if (atomic_cas(&hid_fifo_write_pos, (atomic_val_t)pos, (atomic_val_t)(pos + 1))) {
			break;
		}
	}

	struct hid_fifo_slot *slot = &hid_fifo_slots[pos & HID_FIFO_MASK];
	/*
	 * Single-core: consume publishes seq before read_pos, so capacity check
	 * means slot is ready. Never spin here — producers include ESB EVENT IRQ.
	 */
	memcpy(slot->data, data, sizeof(slot->data));
	slot->subscription_generation = generation;
	atomic_set(&slot->seq, (atomic_val_t)(pos + 1));
	return true;
}

/* Peek without consuming — consume only after USB submit succeeds. */
static size_t hid_fifo_peek_batch(struct tracker_report *out, uint32_t *generations, size_t max)
{
	size_t n = 0;
	uint32_t read = (uint32_t)atomic_get(&hid_fifo_read_pos);

	while (n < max) {
		struct hid_fifo_slot *slot = &hid_fifo_slots[(read + (uint32_t)n) & HID_FIFO_MASK];

		if ((uint32_t)atomic_get(&slot->seq) != read + (uint32_t)n + 1U) {
			break;
		}
		memcpy(out[n].data, slot->data, sizeof(out[n].data));
		generations[n] = slot->subscription_generation;
		n++;
	}
	return n;
}

static void hid_fifo_consume(size_t n)
{
	for (size_t i = 0; i < n; i++) {
		uint32_t read = (uint32_t)atomic_get(&hid_fifo_read_pos);
		struct hid_fifo_slot *slot = &hid_fifo_slots[read & HID_FIFO_MASK];

		atomic_set(&slot->seq, (atomic_val_t)(read + MAX_REPORTS));
		atomic_set(&hid_fifo_read_pos, (atomic_val_t)(read + 1U));
	}
}

static bool hid_fifo_is_empty(void)
{
	uint32_t read = (uint32_t)atomic_get(&hid_fifo_read_pos);
	struct hid_fifo_slot *slot = &hid_fifo_slots[read & HID_FIFO_MASK];

	return (uint32_t)atomic_get(&slot->seq) != read + 1U;
}

static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);
static bool hid_ready;
static uint32_t hid_idle_duration_ms;

#define HID_EP_BUSY_FLAG	0
/* Coalesce newly queued records; retry only while a submission needs service. */
#define REPORT_PERIOD		K_MSEC(1)
#define HID_REGISTRATION_PERIOD K_MSEC(100)
#define HID_EP_REPORT_COUNT 4
#define HID_TPS_UPDATE_INTERVAL_MS 1000
#define HID_STATS_POLL_INTERVAL_MS 200

// EMA平滑因子配置
// alpha = RSSI_EMA_ALPHA / 256
// alpha = 0.2 (51/256 ≈ 0.199) - 相当于约10个样本的简单移动平均
// alpha越小，平滑效果越强但响应越慢
#define RSSI_EMA_ALPHA 51  // 范围: 1-255, 推荐值: 26-77 (0.1-0.3)

struct tracker_report ep_report_buffer[HID_EP_REPORT_COUNT] __aligned(sizeof(void *));

// RSSI EMA平滑处理结构
struct rssi_ema_state {
	int16_t ema_value;    // EMA值，使用int16避免溢出（实际值 * 256）
	bool initialized;     // 是否已初始化
};

static struct rssi_ema_state rssi_states[MAX_TRACKERS] = {0};

struct hid_stats_state {
	atomic_t reports_in_interval;
	atomic_t current_tps;
	atomic_t last_tps_time;
	atomic_t last_report_time;
	atomic_t had_activity;
	atomic_t started;
};

static struct hid_stats_state hid_stats = {
	.reports_in_interval = ATOMIC_INIT(0),
	.current_tps = ATOMIC_INIT(0),
	.last_tps_time = ATOMIC_INIT(0),
	.last_report_time = ATOMIC_INIT(0),
	.had_activity = ATOMIC_INIT(0)
};

LOG_MODULE_REGISTER(hid_event, LOG_LEVEL_INF);

static void report_event_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, report_event_handler, NULL);

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_INPUT(0x02),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_OUTPUT(0x02),
	HID_END_COLLECTION,
};

uint16_t sent_device_addr = 0;
int64_t last_registration_sent = 0;

//|type    |description
//|TX   255|registration: associate tracker id with device address
//|RX   254|dongle command envelope (see rcv_hid_cmd.h)
//|TX   251|dongle command ACK
//|RX/TX 240-247|HID OTA (see esb_ota.h / receiver_ota.h)
// Tracker stream types 0-7 are documented near hid_write_packet_n.

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |data                                                                                                                                  |
//|TX   255|id      |device_addr                                          |resv-------------------------------------------------------------------|
//|TX   251|seq     |opcode  |status  |result---------------------------------------------------------------|
//|RX   254|seq     |opcode  |flags   |args-----------------------------------------------------------------|

static bool packet_device_addr(uint8_t *report, uint16_t id) // associate id and tracker address
{
	unsigned int key = irq_lock();
	if (stored_trackers > MAX_TRACKERS || id >= stored_trackers || id >= MAX_TRACKERS) {
		irq_unlock(key);
		return false;
	}
	memcpy(&report[2], &stored_tracker_addr[id], 6);
	irq_unlock(key);
	report[0] = RCV_HID_TYPE_DEVICE_ADDR;
	report[1] = id;
	memset(&report[8], 0, 8); // last 8 bytes unused for now
	return true;
}

static void hid_stats_record_reports(uint32_t reports)
{
	if (reports == 0U)
		return;

	uint32_t now = k_uptime_get_32();

	if (atomic_cas(&hid_stats.started, 0, 1)) {
		atomic_set(&hid_stats.last_tps_time, (atomic_val_t)now);
	}

	atomic_add(&hid_stats.reports_in_interval, (atomic_val_t)reports);
	atomic_set(&hid_stats.last_report_time, (atomic_val_t)now);
	atomic_set(&hid_stats.had_activity, 1);

	uint32_t last_tps = (uint32_t)atomic_get(&hid_stats.last_tps_time);
	if (now - last_tps >= HID_TPS_UPDATE_INTERVAL_MS) {
		atomic_val_t interval_reports = atomic_set(&hid_stats.reports_in_interval, 0);
		atomic_set(&hid_stats.current_tps, interval_reports);
		atomic_set(&hid_stats.last_tps_time, (atomic_val_t)now);
	}
}

static void hid_stats_update_idle_if_needed(uint32_t now)
{
	uint32_t last_tps = (uint32_t)atomic_get(&hid_stats.last_tps_time);
	if (!atomic_get(&hid_stats.started)) {
		return;
	}

	if (now - last_tps >= HID_TPS_UPDATE_INTERVAL_MS) {
		atomic_val_t interval_reports = atomic_get(&hid_stats.reports_in_interval);
		if (interval_reports > 0) {
			atomic_set(&hid_stats.current_tps, interval_reports);
			atomic_set(&hid_stats.reports_in_interval, 0);
		} else {
			uint32_t last_report = (uint32_t)atomic_get(&hid_stats.last_report_time);
			if (now - last_report >= HID_TPS_UPDATE_INTERVAL_MS) {
				atomic_set(&hid_stats.current_tps, 0);
			}
		}
		atomic_set(&hid_stats.last_tps_time, (atomic_val_t)now);
	}
}

static uint32_t hid_stats_snapshot(bool *had_activity)
{
	uint32_t current_tps = (uint32_t)atomic_get(&hid_stats.current_tps);

	if (had_activity) {
		*had_activity = (bool)atomic_set(&hid_stats.had_activity, 0);
	}

	return current_tps;
}

static uint32_t dropped_reports = 0;
static uint16_t max_dropped_reports = 0;
/* Per-tracker drop counters; printed from logging thread when detailed stats on. */
static uint32_t tracker_drops[MAX_TRACKERS] = {0};
static uint32_t total_dropped_reports = 0;
static uint32_t total_tracker_drops[MAX_TRACKERS] = {0};

uint32_t hid_get_total_drop_count(void)
{
	return total_dropped_reports;
}

uint32_t hid_get_total_tracker_drop_count(uint8_t tracker_id)
{
	return tracker_id < MAX_TRACKERS ? total_tracker_drops[tracker_id] : 0;
}


static void hid_fill_padding(struct tracker_report *out, uint8_t tracker_count)
{
	if (tracker_count > 0 && tracker_count <= MAX_TRACKERS) {
		sent_device_addr %= tracker_count;
		bool valid = packet_device_addr(out->data, sent_device_addr);
		sent_device_addr = (sent_device_addr + 1) % tracker_count;
		if (valid) {
			return;
		}
	}
	memset(out->data, 0, sizeof(out->data));
	out->data[0] = 0xF8;
}

static void send_report(struct k_work *work)
{
	if (!receiver_usb_is_enabled()) return;
	if (!receiver_usb_is_configured()) return;
	if (!hid_ready) return;

	uint8_t tracker_count = stored_trackers;
	bool fifo_empty = hid_fifo_is_empty();
	if (fifo_empty && (tracker_count == 0 || k_uptime_get() - 100 < last_registration_sent)) {
		return; // send registrations only every 100ms when trackers are stored
	}

	int ret;

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		uint32_t generations[HID_EP_REPORT_COUNT];
		size_t reports_to_send = hid_fifo_peek_batch(ep_report_buffer, generations,
							    HID_EP_REPORT_COUNT);

		int epind = (int)reports_to_send;
		for (; epind < HID_EP_REPORT_COUNT; epind++) {
			hid_fill_padding(&ep_report_buffer[epind], tracker_count);
		}

		/* Recheck immediately before submission, including every retry. An
		 * invalid entry is consumed only when its replacement is submitted.
		 */
		uint32_t now_ms = k_uptime_get_32();
		for (size_t i = 0; i < reports_to_send; i++) {
			if (generations[i] != 0 &&
			    !tracker_events_hid_valid(ep_report_buffer[i].data, generations[i], now_ms)) {
				hid_fill_padding(&ep_report_buffer[i], tracker_count);
			}
		}

		ret = hid_device_submit_report(hdev, sizeof(report) * HID_EP_REPORT_COUNT,
					       (uint8_t *)ep_report_buffer);

		if (ret != 0) {
			k_work_schedule(&report_send, REPORT_PERIOD);
			atomic_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
			static int64_t last_err_log;
			int64_t now = k_uptime_get();
			if (ret != -EACCES && now - last_err_log > 5000) {
				LOG_ERR("Failed to submit report: %d", ret);
				last_err_log = now;
			}
		} else {
			hid_fifo_consume(reports_to_send);
			last_registration_sent = k_uptime_get();
			/* Anchor idle registration to the last successful report, not to
			 * an unrelated periodic phase after streaming becomes idle.
			 */
			k_timer_start(&event_timer, HID_REGISTRATION_PERIOD, HID_REGISTRATION_PERIOD);
			if (reports_to_send > 0U) {
				hid_stats_record_reports((uint32_t)reports_to_send);
			}
		}
	}
}

#define DROPPED_REPORT_LOG_INTERVAL 1000  // Log every 1 second when stats enabled

static void hid_dropped_reports_logging(void)
{
	uint32_t last_logged_tps = UINT32_MAX;
	int64_t last_log_time = k_uptime_get();

	while (1) {
		k_msleep(HID_STATS_POLL_INTERVAL_MS);

		int64_t now = k_uptime_get();
		hid_stats_update_idle_if_needed(k_uptime_get_32());

		// Only log when detailed stats are enabled
		if (!esb_get_stats_detailed_enabled()) {
			last_log_time = now;
			continue;
		}

		if (now - last_log_time >= DROPPED_REPORT_LOG_INTERVAL) {
			bool had_activity = false;
			uint32_t current_tps = hid_stats_snapshot(&had_activity);

			if (dropped_reports)
				LOG_INF("Dropped reports: %u (max: %u)", dropped_reports, max_dropped_reports);
			dropped_reports = 0;
			max_dropped_reports = 0;

			for (int i = 0; i < MAX_TRACKERS; i++) {
				if (tracker_drops[i]) {
					LOG_INF("HID drops trk %u: %u", i, tracker_drops[i]);
					tracker_drops[i] = 0;
				}
			}

			if (had_activity && current_tps != last_logged_tps) {
				LOG_INF("HID TPS: %u", current_tps);
				last_logged_tps = current_tps;
			} else if (!had_activity) {
				last_logged_tps = UINT32_MAX;
			}

			last_log_time = now;
		}
	}
}

uint32_t hid_get_current_tps(void)
{
	return hid_stats_snapshot(NULL);
}

// RSSI指数加权移动平均滤波函数
// 使用EMA算法: EMA(n) = alpha * X(n) + (1 - alpha) * EMA(n-1)
// 为避免浮点运算，使用定点数: alpha = RSSI_EMA_ALPHA / 256
static uint8_t rssi_smooth_update(uint8_t tracker_id, int8_t new_rssi)
{
	if (tracker_id >= MAX_TRACKERS) {
		return (uint8_t)new_rssi;
	}

	struct rssi_ema_state *state = &rssi_states[tracker_id];

	// 首次初始化，直接使用当前值
	if (!state->initialized) {
		state->ema_value = (int16_t)new_rssi * 256;  // Q8 fixed point
		state->initialized = true;
		return (uint8_t)new_rssi;
	}

	// EMA计算: EMA = alpha * new + (1 - alpha) * old_EMA
	// 定点数实现: EMA = (alpha * new * 256 + (256 - alpha) * old_EMA) / 256
	int32_t new_scaled = (int32_t)new_rssi * 256;
	int32_t alpha_term = (int32_t)RSSI_EMA_ALPHA * new_scaled;  // alpha * new * 256
	int32_t beta_term = (256 - RSSI_EMA_ALPHA) * state->ema_value;  // (1-alpha) * old_EMA

	state->ema_value = (int16_t)((alpha_term + beta_term) >> 8);  // 除以256

	// 返回EMA值（右移8位还原）
	int8_t result = (int8_t)(state->ema_value >> 8);
	return (uint8_t)result;
}

// 重置特定tracker的RSSI平滑状态
void hid_reset_rssi_smooth(uint8_t tracker_id)
{
	if (tracker_id < MAX_TRACKERS) {
		rssi_states[tracker_id].initialized = false;
		rssi_states[tracker_id].ema_value = 0;
	}
}

/* Below ESB_THREAD_PRIORITY; must not compete with radio housekeeping. */
K_THREAD_DEFINE(hid_dropped_reports_logging_thread, 256, hid_dropped_reports_logging, NULL, NULL, NULL, HID_DROPPED_REPORTS_LOGGING_PRIORITY, 0, 0);

static void handle_output_report(const uint8_t *buf, uint16_t len, uint32_t usb_generation)
{
	if (len == 0) {
		return;
	}

	uint8_t report_type = buf[0];
	if (report_type >= 0xF0 && report_type <= 0xF7) {
		if (len >= 2 && buf[1] == RECEIVER_OTA_ID) {
			receiver_ota_process_hid(buf, len);
		} else {
			esb_ota_relay_process_hid(buf, len);
		}
		return;
	}

	if (report_type == RCV_HID_TYPE_CMD) {
		uint8_t ack[RCV_HID_CMD_LEN];
		if (rcv_cmd_process_hid(buf, len, usb_generation, ack)) {
			hid_write_packet_n(ack, 0);
		}
	}
}

/* CMD path can printk / take mutex / clear trackers — keep off USB callback. */
struct hid_cmd_msg {
	uint8_t len;
	uint8_t data[RCV_HID_CMD_LEN];
	uint32_t usb_generation;
};

K_MSGQ_DEFINE(hid_cmd_msgq, sizeof(struct hid_cmd_msg), 4, 4);

static void hid_cmd_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	struct hid_cmd_msg msg;

	while (k_msgq_get(&hid_cmd_msgq, &msg, K_NO_WAIT) == 0) {
		handle_output_report(msg.data, msg.len, msg.usb_generation);
	}
}

static K_WORK_DEFINE(hid_cmd_work, hid_cmd_work_handler);

static void enqueue_hid_cmd(const uint8_t *buf, uint16_t len)
{
	struct hid_cmd_msg msg;

	if (len < 1) {
		return;
	}
	msg.len = (uint8_t)MIN(len, sizeof(msg.data));
	msg.usb_generation = tracker_events_usb_generation();
	memcpy(msg.data, buf, msg.len);
	if (k_msgq_put(&hid_cmd_msgq, &msg, K_NO_WAIT) != 0) {
		LOG_WRN("HID CMD queue full, dropping");
		return;
	}
	k_work_submit(&hid_cmd_work);
}

static void dispatch_output_report(const uint8_t *buf, uint16_t len)
{
	if (len == 0 || buf == NULL) {
		return;
	}

	/* OTA DATA needs throughput: stay synchronous. CMD: defer. */
	if (buf[0] == RCV_HID_TYPE_CMD) {
		enqueue_hid_cmd(buf, len);
		return;
	}
	handle_output_report(buf, len, tracker_events_usb_generation());
}

static void int_in_ready_cb(const struct device *dev)
{
	if (dev != hdev) {
		return;
	}

	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		if (receiver_usb_is_configured() && hid_ready) {
			LOG_WRN("IN endpoint callback without preceding buffer write");
		}
	}
	if (!hid_fifo_is_empty()) {
		/* Backlog is already coalesced. Do not add another USB frame of
		 * holdoff, even if a producer armed a later batching deadline.
		 */
		k_work_reschedule(&report_send, K_NO_WAIT);
	}
}

static void input_report_done_cb(const struct device *dev, const uint8_t *const submitted_report)
{
	if (dev != hdev) {
		return;
	}

	if (submitted_report != (const uint8_t *)ep_report_buffer &&
	    receiver_usb_is_configured() && hid_ready) {
		LOG_WRN("IN endpoint callback for unexpected report buffer");
	}

	int_in_ready_cb(dev);
}

static void iface_ready_cb(const struct device *dev, const bool ready)
{
	if (dev != hdev) {
		return;
	}

	hid_ready = ready;
	if (!ready) {
		tracker_events_usb_reset();
		atomic_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	} else {
		k_work_schedule(&report_send, REPORT_PERIOD);
	}
}

static int get_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
			 const uint16_t len, uint8_t *const buf)
{
	if (dev != hdev) {
		return -ENODEV;
	}

	if (type != HID_REPORT_TYPE_INPUT || id != 0U) {
		return -ENOTSUP;
	}

	if (len < sizeof(ep_report_buffer) || buf == NULL) {
		return -EINVAL;
	}

	memset(buf, 0, sizeof(ep_report_buffer));
	return sizeof(ep_report_buffer);
}

static int set_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
			 const uint16_t len, const uint8_t *const buf)
{
	if (dev != hdev) {
		return -ENODEV;
	}

	if (type != HID_REPORT_TYPE_OUTPUT || id != 0U) {
		return -ENOTSUP;
	}

	if (len == 0U || len > 64U || buf == NULL) {
		return -EINVAL;
	}

	dispatch_output_report(buf, len);
	return 0;
}

static void on_idle_cb(const struct device *dev, const uint8_t report_id, const uint32_t duration)
{
	if (dev != hdev || report_id != 0U) {
		return;
	}

	hid_idle_duration_ms = duration;
	k_work_schedule(&report_send, REPORT_PERIOD);
}

static uint32_t get_idle_cb(const struct device *dev, const uint8_t report_id)
{
	if (dev != hdev || report_id != 0U) {
		return 0;
	}

	return hid_idle_duration_ms;
}

static void output_report_cb(const struct device *dev, const uint16_t len, const uint8_t *const buf)
{
	if (dev != hdev || len == 0U || len > 64U || buf == NULL) {
		return;
	}

	dispatch_output_report(buf, len);
}

static void report_event_handler(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);
	/* Registration heartbeat is the only periodic sender wakeup. */
	if (stored_trackers > 0 && receiver_usb_is_enabled() &&
	    receiver_usb_is_configured() && hid_ready) {
		k_work_schedule(&report_send, K_NO_WAIT);
	}
}

static const struct hid_device_ops ops = {
	.iface_ready = iface_ready_cb,
	.get_report = get_report_cb,
	.set_report = set_report_cb,
	.set_idle = on_idle_cb,
	.get_idle = get_idle_cb,
	.input_report_done = input_report_done_cb,
	.output_report = output_report_cb,
};

static void hid_usb_state_changed(bool configured)
{
	if (!configured) {
		tracker_events_usb_reset();
		hid_ready = false;
		atomic_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	} else {
		k_work_schedule(&report_send, REPORT_PERIOD);
	}
}

static void hid_async_cmd_ack(const uint8_t ack[RCV_HID_CMD_LEN])
{
	hid_write_packet_n(ack, 0);
}

static int composite_pre_init(void)
{
	hdev = DEVICE_DT_GET(DT_NODELABEL(hid_dev_0));
	if (!device_is_ready(hdev)) {
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_INF("HID Device: dev %p", hdev);

	int ret = hid_device_register(hdev, hid_report_desc, sizeof(hid_report_desc), &ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID device: %d", ret);
		return ret;
	}

	hid_fifo_init();
	k_work_init_delayable(&report_send, send_report);
	k_timer_start(&event_timer, HID_REGISTRATION_PERIOD, HID_REGISTRATION_PERIOD);
	receiver_usb_set_state_callback(hid_usb_state_changed);
	rcv_cmd_set_async_ack(hid_async_cmd_ack);

	return 0;
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

//|type    |description
//|RX     0|device info ("info")
//|RX     1|full precision quat and accel
//|RX     2|reduced precision quat and accel with battery, temp, and rssi ("info")
//|RX     3|status ("status")
//|RX     4|full precision quat and magnetometer
//|RX     5|runtime ("status2")
//|RX     6|reduced precision quat and accel with button and sleep time ("info2")
//|RX     7|button and sleep time ("info2")

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|RX     0|id      |batt    |batt_v  |temp    |brd_id  |mcu_id  |resv----|imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|RX     1|id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|RX     2|id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|RX     3|id      |svr_stat|status  |resv----------------------------------------------------------------------------------------------|rssi    |
//|RX     4|id      |q0               |q1               |q2               |q3               |m0               |m1               |m2               |
//|RX     5|id      |runtime                                                                |resv----------------------------------------|rssi    |
//|RX     6|id      |button  |sleeptime        |resv-------------------------------------------------------------------------------------|rssi    |
//|RX     7|id      |button  |sleeptime        |q_buf                              |a0               |a1               |a2               |rssi    |

// runtime is in microseconds (overkill), sleeptime is in milliseconds (overkill but less)

/* byte[1] is a tracker id for stream/registration types, not for cmd/ACK. */
static bool hid_type_byte1_is_tracker_id(uint8_t type)
{
	return type != RCV_HID_TYPE_CMD_ACK && type != RCV_HID_TYPE_CMD;
}

/* Types that reserve byte[15] for RSSI (not full-quat or OTA payloads). */
static bool hid_type_has_rssi_slot(uint8_t type)
{
	if (type == 1 || type == 4) {
		return false;
	}
	if (type >= 0xF0 && type <= 0xF7) {
		return false;
	}
	return hid_type_byte1_is_tracker_id(type);
}

static bool hid_enqueue_packet(const uint8_t data[16], uint32_t generation)
{
	if (hid_fifo_try_push(data, hid_fifo_is_priority(data[0]), generation)) {
		/* Schedule, not reschedule: continuous producers cannot postpone TX. */
		k_work_schedule(&report_send, REPORT_PERIOD);
		return true;
	}

	/* Count only; producers may run in the radio IRQ. */
	total_dropped_reports++;
	dropped_reports++;
	if (dropped_reports > max_dropped_reports) {
		max_dropped_reports = dropped_reports;
	}
	if (hid_type_byte1_is_tracker_id(data[0])) {
		uint8_t tracker_id = data[1];
		if (tracker_id < MAX_TRACKERS) {
			tracker_drops[tracker_id]++;
			total_tracker_drops[tracker_id]++;
		}
	}
	return false;
}

bool hid_write_tracker_event(const uint8_t record[16], uint32_t generation)
{
	if (generation == 0) {
		return false;
	}
	/* Type 251 already has priority and no RSSI slot: preserve all 16 bytes. */
	return hid_enqueue_packet(record, generation);
}

void hid_write_packet_n(const uint8_t *data, uint8_t rssi)
{
	uint8_t pkt[16];

	// Drop packets with all-zero quaternion (type 1 and 4: quat in bytes 2-9).
	if (data[0] == 1 || data[0] == 4) {
		const uint16_t *q = (const uint16_t *)&data[2];
		if (q[0] == 0 && q[1] == 0 && q[2] == 0 && q[3] == 0) {
			return;
		}
	}

	memcpy(pkt, data, sizeof(pkt));
	if (hid_type_has_rssi_slot(data[0])) {
		uint8_t tracker_id = data[1];
		if (tracker_id < MAX_TRACKERS) {
			pkt[15] = rssi_smooth_update(tracker_id, (int8_t)rssi);
		} else {
			pkt[15] = rssi;
		}
	}

	(void)hid_enqueue_packet(pkt, 0);
}
