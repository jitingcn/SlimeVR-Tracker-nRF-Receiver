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

#include <limits.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

static struct k_work report_send;
static struct k_work report_read;

static struct tracker_report {
	uint8_t data[16];
} __packed report = {
	.data = {0}
};;


#define MAX_REPORTS (MAX_TRACKERS * 4)

struct tracker_report reports[MAX_REPORTS];
atomic_t report_write_index = 0;
atomic_t report_read_index = 0;
// read_index == write_index -> empty fifo
// (write_index + 1) % MAX_REPORTS == read_index -> full fifo

static bool configured;
static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);
static ATOMIC_DEFINE(hid_ep_out_busy, 1);

#define HID_EP_BUSY_FLAG	0
#define REPORT_PERIOD		K_MSEC(1) // streaming reports
#define POLL_PERIOD		K_MSEC(1)
#define HID_EP_REPORT_COUNT 4
#define HID_TPS_UPDATE_INTERVAL_MS 1000
#define HID_STATS_POLL_INTERVAL_MS 200
#define USB_EP_TIMEOUT_MS 100  // USB endpoint timeout threshold

// EMA平滑因子配置
// alpha = RSSI_EMA_ALPHA / 256
// alpha = 0.2 (51/256 ≈ 0.199) - 相当于约10个样本的简单移动平均
// alpha越小，平滑效果越强但响应越慢
#define RSSI_EMA_ALPHA 51  // 范围: 1-255, 推荐值: 26-77 (0.1-0.3)

struct tracker_report ep_report_buffer[HID_EP_REPORT_COUNT];
static uint8_t ep_read_buffer[256];

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
bool usb_enabled = false;
int64_t last_registration_sent = 0;

//|type    |description
//|TX   255|receiver packet 0, associate id and tracker address
//|TX   254|device list hash, round trip ping
//|TX   253|receiver status
//|TX   252|device pairing request
//|RX   255|round trip response
//|RX   254|command

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |data                                                                                                                                  |
//|TX   255|id      |device_addr                                          |resv-------------------------------------------------------------------|
//|TX   254|stored  |crc                                |latency          |resv-------------------------------------------------------------------|
//|TX   253|status  |resv-------------------------------------------------|resv-------------------------------------------------------------------|
//|TX   252|id      |device_addr                                          |resv-------------------------------------------------------------------|
//|RX   255|resv----------------------------------------------------------|resv-------------------------------------------------------------------|
//|RX   254|command |resv-------------------------------------------------|resv-------------------------------------------------------------------|

static void packet_device_addr(uint8_t *report, uint16_t id) // associate id and tracker address
{
	report[0] = 255; // receiver packet 0
	report[1] = id;
	memcpy(&report[2], &stored_tracker_addr[id], 6);
	memset(&report[8], 0, 8); // last 8 bytes unused for now
}

static void hid_stats_record_reports(uint32_t reports)
{
	if (reports == 0U)
		return;

	int64_t now = k_uptime_get();

	atomic_val_t last_tps = atomic_get(&hid_stats.last_tps_time);
	if (last_tps == 0) {
		atomic_cas(&hid_stats.last_tps_time, 0, now);
	}

	atomic_add(&hid_stats.reports_in_interval, (atomic_val_t)reports);
	atomic_set(&hid_stats.last_report_time, now);
	atomic_set(&hid_stats.had_activity, 1);

	last_tps = atomic_get(&hid_stats.last_tps_time);
	if (now - last_tps >= HID_TPS_UPDATE_INTERVAL_MS) {
		atomic_val_t interval_reports = atomic_set(&hid_stats.reports_in_interval, 0);
		atomic_set(&hid_stats.current_tps, interval_reports);
		atomic_set(&hid_stats.last_tps_time, now);
	}
}

static void hid_stats_update_idle_if_needed(int64_t now)
{
	atomic_val_t last_tps = atomic_get(&hid_stats.last_tps_time);
	if (last_tps == 0) {
		return;
	}

	if (now - last_tps >= HID_TPS_UPDATE_INTERVAL_MS) {
		atomic_val_t interval_reports = atomic_get(&hid_stats.reports_in_interval);
		if (interval_reports > 0) {
			atomic_set(&hid_stats.current_tps, interval_reports);
			atomic_set(&hid_stats.reports_in_interval, 0);
		} else {
			atomic_val_t last_report = atomic_get(&hid_stats.last_report_time);
			if (last_report && now - last_report >= HID_TPS_UPDATE_INTERVAL_MS) {
				atomic_set(&hid_stats.current_tps, 0);
			}
		}
		atomic_set(&hid_stats.last_tps_time, now);
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
static int64_t last_ep_busy_time = 0;  // Track USB endpoint busy time for timeout detection
static int64_t ep_cooldown_until = 0;  // After stuck reset, wait before retrying

static void send_report(struct k_work *work)
{
	if (!usb_enabled) return;
	if (!configured) return;  // Don't send reports until USB is configured
	if (!stored_trackers) return;

	// After a stuck reset, wait for cooldown before retrying
	if (ep_cooldown_until > 0) {
		if (k_uptime_get() < ep_cooldown_until) {
			return;
		}
		ep_cooldown_until = 0;
	}

	// Check if USB endpoint is stuck
	if (atomic_test_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		int64_t now = k_uptime_get();
		if (last_ep_busy_time == 0) {
			last_ep_busy_time = now;
		} else if (now - last_ep_busy_time > USB_EP_TIMEOUT_MS) {
			LOG_WRN("USB endpoint stuck for %lld ms, forcing reset", now - last_ep_busy_time);
			atomic_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
			last_ep_busy_time = 0;
			ep_cooldown_until = now + 50;  // 50ms cooldown for USB to settle
			return;
		}
	} else {
		last_ep_busy_time = 0;
	}

	// Get current FIFO status atomically
	size_t write_idx = (size_t)atomic_get(&report_write_index);
	size_t read_idx = (size_t)atomic_get(&report_read_index);

	if (write_idx == read_idx && k_uptime_get() - 100 < last_registration_sent) {
		return; // send registrations only every 100ms
	}

	int ret, wrote;

	last_registration_sent = k_uptime_get();

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		// Calculate how many reports we have available
		int available_reports = write_idx - read_idx;
		if (available_reports < 0) available_reports += MAX_REPORTS;
		size_t reports_to_send = (size_t) MIN(available_reports, HID_EP_REPORT_COUNT);

		int epind;
		// Copy existing data to buffer
		for (epind = 0; epind < reports_to_send; epind++) {
			ep_report_buffer[epind] = reports[read_idx];
			read_idx++;
			if (read_idx == MAX_REPORTS) {
				read_idx = 0;
			}
		}

		if (reports_to_send > 0U) {
			atomic_set(&report_read_index, read_idx);
			hid_stats_record_reports((uint32_t)reports_to_send);
		}

		// Pad remaining report slots with device addr
		for (; epind < HID_EP_REPORT_COUNT; epind++) {
			if (stored_trackers > 0) {
				packet_device_addr(ep_report_buffer[epind].data, sent_device_addr);
				sent_device_addr = (sent_device_addr + 1) % stored_trackers;
			}
		}

		ret = hid_int_ep_write(hdev, (uint8_t *)ep_report_buffer, sizeof(report) * HID_EP_REPORT_COUNT, &wrote);

		if (ret != 0) {
			/*
			 * Write failed — clear the busy flag immediately so the
			 * next timer cycle can attempt to send again.  Without
			 * this, the flag stays set until the 100 ms stuck timeout
			 * because the completion callback never fires on error.
			 */
			atomic_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
			static int64_t last_err_log;
			int64_t now = k_uptime_get();
			if (now - last_err_log > 5000) {
				LOG_ERR("Failed to submit report");
				last_err_log = now;
			}
		} else {
			//LOG_DBG("Report submitted");
		}
	} else { // busy with what
		//LOG_DBG("HID IN endpoint busy");
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
		hid_stats_update_idle_if_needed(now);

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
		state->ema_value = (int16_t)new_rssi << 8;  // 左移8位作为定点数
		state->initialized = true;
		return (uint8_t)new_rssi;
	}

	// EMA计算: EMA = alpha * new + (1 - alpha) * old_EMA
	// 定点数实现: EMA = (alpha * new * 256 + (256 - alpha) * old_EMA) / 256
	int32_t new_scaled = (int32_t)new_rssi << 8;  // new * 256
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

// 重置所有tracker的RSSI平滑状态
void hid_reset_all_rssi_smooth(void)
{
	memset(rssi_states, 0, sizeof(rssi_states));
}

K_THREAD_DEFINE(hid_dropped_reports_logging_thread, 256, hid_dropped_reports_logging, NULL, NULL, NULL, 7, 0, 0);

static void read_report(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!usb_enabled || !configured) {
		return;
	}

	int ret;
	int read = 0;

	if (!atomic_test_and_set_bit(hid_ep_out_busy, HID_EP_BUSY_FLAG)) {
		ret = hid_int_ep_read(hdev, ep_read_buffer, sizeof(ep_read_buffer), &read);
		if (ret != 0) {
			LOG_ERR("hid_int_ep_read: %d", ret);
		} else if (read > 0) {
			/* Dispatch HID OUT reports */
			uint8_t report_type = ep_read_buffer[0];
			if (report_type >= 0xF0 && report_type <= 0xF7) {
				/* OTA command from PC */
				esb_ota_relay_process_hid(ep_read_buffer, read);
			}
		}
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		// During cooldown after stuck reset, stale callbacks are expected
		if (ep_cooldown_until == 0) {
			LOG_WRN("IN endpoint callback without preceding buffer write");
		}
	}
}

static void int_out_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_out_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("OUT endpoint callback without preceding buffer read");
	}
}

/*
 * On Idle callback is available here as an example even if actual use is
 * very limited. In contrast to report_event_handler(),
 * report value is not incremented here.
 */
static void on_idle_cb(const struct device *dev, uint16_t report_id)
{
	LOG_DBG("On idle callback");
	k_work_submit(&report_send);
}

static void report_event_handler(struct k_timer *dummy)
{
	if (usb_enabled)
		k_work_submit(&report_send);
}

static void report_read_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(read_timer, report_read_handler, NULL);

static void report_read_handler(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);
	if (usb_enabled)
		k_work_submit(&report_read);
}

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
	.int_out_ready = int_out_ready_cb,
	.on_idle = on_idle_cb,
	.protocol_change = protocol_cb,
};

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status) {
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONFIGURED:
		int configurationIndex = *param;
		if(configurationIndex == 0) {
			// from usb_device.c: A configuration index of 0 unconfigures the device.
			configured = false;
		} else {
			if (!configured) {
				int_in_ready_cb(hdev);
				configured = true;
			}
		}
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static int composite_pre_init()
{
	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_INF("HID Device: dev %p", hdev);

	usb_hid_register_device(hdev, hid_report_desc, sizeof(hid_report_desc),
				&ops);

	atomic_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	k_timer_start(&event_timer, REPORT_PERIOD, REPORT_PERIOD);
	atomic_set_bit(hid_ep_out_busy, HID_EP_BUSY_FLAG);
	k_timer_start(&read_timer, POLL_PERIOD, POLL_PERIOD);

	if (usb_hid_set_proto_code(hdev, HID_BOOT_IFACE_CODE_NONE)) {
		LOG_WRN("Failed to set Protocol Code");
	}

	return usb_hid_init(hdev);
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void usb_init_thread(void)
{
	usb_enable(status_cb);
	k_work_init(&report_send, send_report);
	k_work_init(&report_read, read_report);
	usb_enabled = true;
}

K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 0);

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

// Per-tracker FIFO drop tracking for detailed diagnostics
static uint32_t tracker_drops[MAX_TRACKERS] = {0};
static int64_t last_drop_log_time[MAX_TRACKERS] = {0};
#define TRACKER_DROP_LOG_INTERVAL_MS 1000  // Log per-tracker drops every 1s

void hid_write_packet_n(uint8_t *data, uint8_t rssi)
{
	// Drop packets with all-zero quaternion (type 1 and 4: quat in bytes 2-9).
	if (data[0] == 1 || data[0] == 4) {
		const uint16_t *q = (const uint16_t *)&data[2];
		if (q[0] == 0 && q[1] == 0 && q[2] == 0 && q[3] == 0) {
			return;
		}
	}

	memcpy(&report.data, data, sizeof(report)); // all data can be passed through

	/* Types 1, 4: full-precision quat+accel — no room for RSSI.
	 * Types 0xF0-0xF7: OTA reports use all 16 bytes — no room for RSSI. */
	if (data[0] != 1 && data[0] != 4 &&
	    !(data[0] >= 0xF0 && data[0] <= 0xF7)) {
		uint8_t tracker_id = data[1];
		uint8_t smoothed_rssi = rssi_smooth_update(tracker_id, (int8_t)rssi);
		report.data[15] = smoothed_rssi;
	}

	// Get current FIFO status atomically
	size_t write_idx = (size_t)atomic_get(&report_write_index);
	size_t read_idx = (size_t)atomic_get(&report_read_index);

	// Calculate next write position
	size_t next_write = write_idx + 1;
	if (next_write == MAX_REPORTS) next_write = 0;

	// Check if FIFO is full
	if (next_write == read_idx) {
		dropped_reports++;
		if (dropped_reports > max_dropped_reports) {
			max_dropped_reports = dropped_reports;
		}

		// Per-tracker drop tracking and logging
		uint8_t tracker_id = data[1];
		if (tracker_id < MAX_TRACKERS) {
			tracker_drops[tracker_id]++;
			int64_t now = k_uptime_get();
			if (now - last_drop_log_time[tracker_id] >= TRACKER_DROP_LOG_INTERVAL_MS) {
				LOG_WRN("FIFO full: dropped %u packets for tracker %u (write=%zu read=%zu)",
						tracker_drops[tracker_id], tracker_id, write_idx, read_idx);
				last_drop_log_time[tracker_id] = now;
				tracker_drops[tracker_id] = 0;
			}
		}
		return;
	}

	// Write new packet into FIFO
	reports[write_idx] = report;

	// Update write index atomically
	atomic_set(&report_write_index, next_write);
}
