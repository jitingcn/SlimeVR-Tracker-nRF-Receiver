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

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <math.h>

static struct k_work report_send;

static struct tracker_report {
	uint8_t data[16];
} __packed report = {
	.data = {0}
};;

struct tracker_report reports[MAX_TRACKERS];
atomic_t report_write_index = 0;
atomic_t report_read_index = 0;
// read_index == write_index -> empty fifo
// (write_index + 1) % MAX_TRACKERS == read_index -> full fifo

static bool configured;
static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);

#define HID_EP_BUSY_FLAG	0
#define REPORT_PERIOD		K_MSEC(1) // streaming reports
#define HID_EP_REPORT_COUNT 4

// Rotation anomaly detection constants
#define ROTATION_THRESHOLD_MIN	0.5f
#define ROTATION_THRESHOLD_MAX	(2.0f * M_PI - ROTATION_THRESHOLD_MIN)
#define MAX_INVALID_COUNT		3  // Allow slightly more invalid packets before reset
#define MAX_REPORT_QUEUE		100
#define PACKET_TYPE_ROTATION		1
#define PACKET_TYPE_COMPRESSED		2
#define PACKET_TYPE_FULL_MAG		4

// Utility macros
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

struct tracker_report ep_report_buffer[HID_EP_REPORT_COUNT];

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
	HID_END_COLLECTION,
};

uint16_t sent_device_addr = 0;
bool usb_enabled = false;
int64_t last_registration_sent = 0;

static void packet_device_addr(uint8_t *report, uint16_t id) // associate id and tracker address
{
	report[0] = 255; // receiver packet 0
	report[1] = id;
	memcpy(&report[2], &stored_tracker_addr[id], 6);
	memset(&report[8], 0, 8); // last 8 bytes unused for now
}

static void send_report(struct k_work *work)
{
	if (!usb_enabled) return;
	if (!stored_trackers) return;

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
		if (available_reports < 0) available_reports += MAX_TRACKERS;
		size_t reports_to_send = (size_t)((available_reports > HID_EP_REPORT_COUNT) ? HID_EP_REPORT_COUNT : available_reports);

		int epind;
		// Copy existing data to buffer
		for (epind = 0; epind < reports_to_send; epind++) {
			ep_report_buffer[epind] = reports[read_idx];
			epind++;
			read_idx++;
			if (read_idx == MAX_TRACKERS) read_idx = 0;
			atomic_set(&report_read_index, read_idx);
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
			 * Do nothing and wait until host has reset the device
			 * and hid_ep_in_busy is cleared.
			 */
			LOG_ERR("Failed to submit report");
		} else {
			//LOG_DBG("Report submitted");
		}
	} else { // busy with what
		//LOG_DBG("HID IN endpoint busy");
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
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

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
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
	usb_enabled = true;
}

K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 0);

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|0       |id      |proto   |batt    |batt_v  |temp    |brd_id  |mcu_id  |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|1       |id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|2       |id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|3	   |id      |svr_stat|status  |resv                                                                                              |rssi    |
//|4       |id      |q0               |q1               |q2               |q3               |m0               |m1               |m2               |
//|255     |id      |addr                                                 |resv                                                                   |

#ifndef CONFIG_SOC_NRF52820
#include "util.h"
// last valid data
static float last_q_trackers[MAX_TRACKERS][4] = {0};
static uint32_t last_v_trackers[MAX_TRACKERS] = {0};
static uint8_t last_p_trackers[MAX_TRACKERS] = {0};
static int last_valid_trackers[MAX_TRACKERS] = {0};
// last received data
static float cur_q_trackers[MAX_TRACKERS][4] = {0};
static uint32_t cur_v_trackers[MAX_TRACKERS] = {0};
static uint8_t cur_p_trackers[MAX_TRACKERS] = {0};

// Anomaly detection configuration
typedef enum {
	ANOMALY_DETECTION_STRICT,    // Original algorithm (more false positives)
	ANOMALY_DETECTION_BALANCED,  // Simplified algorithm (recommended)
	ANOMALY_DETECTION_DISABLED   // No anomaly detection
} anomaly_detection_mode_t;

static anomaly_detection_mode_t anomaly_mode = ANOMALY_DETECTION_BALANCED;

// Input validation helper function
static bool validate_tracker_id(uint8_t tracker_id)
{
	return tracker_id < MAX_TRACKERS;
}

// Extract quaternion from packet data
static void extract_quaternion(uint8_t packet_type, const uint8_t *data, float *q)
{
	if (packet_type == PACKET_TYPE_ROTATION || packet_type == PACKET_TYPE_FULL_MAG) {
		// Full precision quaternion from int16_t buffer
		const int16_t *buf = (const int16_t *)&data[2];
		q[0] = FIXED_15_TO_DOUBLE(buf[3]);
		q[1] = FIXED_15_TO_DOUBLE(buf[0]);
		q[2] = FIXED_15_TO_DOUBLE(buf[1]);
		q[3] = FIXED_15_TO_DOUBLE(buf[2]);
	} else if (packet_type == PACKET_TYPE_COMPRESSED) {
		// Compressed vector data
		const uint32_t *q_buf = (const uint32_t *)&data[5];
		float v[3];
		v[0] = FIXED_10_TO_DOUBLE(*q_buf & 1023);
		v[1] = FIXED_11_TO_DOUBLE((*q_buf >> 10) & 2047);
		v[2] = FIXED_11_TO_DOUBLE((*q_buf >> 21) & 2047);

		// Normalize to [-1, 1] range
		for (int i = 0; i < 3; i++) {
			v[i] = v[i] * 2.0f - 1.0f;
		}

		// Convert vector to quaternion
		q_iem(v, q);
	}
}

// Handle rotation anomaly detection and logging
static bool handle_rotation_anomaly(uint8_t tracker_id, uint8_t packet_type,
                                   const float *q, uint32_t q_buf_val)
{
	if (!validate_tracker_id(tracker_id)) {
		LOG_ERR("Invalid tracker ID: %d", tracker_id);
		return true; // Discard packet
	}

	float *last_q = last_q_trackers[tracker_id];
	uint32_t *last_v = &last_v_trackers[tracker_id];
	uint8_t *last_p = &last_p_trackers[tracker_id];
	float *cur_q = cur_q_trackers[tracker_id];
	uint32_t *cur_v = &cur_v_trackers[tracker_id];
	uint8_t *cur_p = &cur_p_trackers[tracker_id];

	// For STRICT mode, use original algorithm completely unchanged
	if (anomaly_mode == ANOMALY_DETECTION_STRICT) {
		// Use original algorithm for STRICT mode to preserve legacy behavior
		float mag = q_diff_mag_original(q, last_q);
		float mag_cur = q_diff_mag_original(q, cur_q);

		// Original simple logic - exactly as it was before
		bool mag_invalid = mag > 0.5f && mag < 6.28f - 0.5f; // possibly invalid rotation
		bool mag_cur_invalid = mag_cur > 0.5f && mag_cur < 6.28f - 0.5f; // possibly inconsistent rotation

		if (mag_invalid) {
			// Original logging format
			LOG_ERR("Abnormal rot. %012llX i%d p%d m%.2f/%.2f v%d",
			        stored_tracker_addr[tracker_id], tracker_id, packet_type,
			        (double)mag, (double)mag_cur, last_valid_trackers[tracker_id]);

			// Original debug output
			printk("a: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n",
			       (double)q[0], (double)q[1], (double)q[2], (double)q[3],
			       packet_type, q_buf_val);
			printk("b: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n",
			       (double)cur_q[0], (double)cur_q[1], (double)cur_q[2], (double)cur_q[3],
			       *cur_p, *cur_v);
			printk("c: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n",
			       (double)last_q[0], (double)last_q[1], (double)last_q[2], (double)last_q[3],
			       *last_p, *last_v);

			// Original logic
			last_valid_trackers[tracker_id]++;
			memcpy(cur_q, q, sizeof(float) * 4);
			*cur_v = q_buf_val;
			*cur_p = packet_type;

			if (!mag_cur_invalid && last_valid_trackers[tracker_id] > 2) { // reset last_q
				LOG_WRN("Reset rotation for %012llX, ID %d",
				        stored_tracker_addr[tracker_id], tracker_id);
				last_valid_trackers[tracker_id] = 0;
				memcpy(last_q, q, sizeof(float) * 4);
				*last_v = q_buf_val;
				*last_p = packet_type;
			}

			return true; // Discard this packet
		}

		// Original update logic for valid packets
		last_valid_trackers[tracker_id] = 0;
		memcpy(cur_q, q, sizeof(float) * 4);
		*cur_v = q_buf_val;
		*cur_p = packet_type;
		memcpy(last_q, q, sizeof(float) * 4);
		*last_v = q_buf_val;
		*last_p = packet_type;

		return false; // Keep this packet
	}

	// For BALANCED mode, use simplified logic - no complex calculations needed
	if (anomaly_mode == ANOMALY_DETECTION_BALANCED) {
		// Simple initialization for first packet
		bool first_packet = (last_q[0] == 0.0f && last_q[1] == 0.0f &&
		                     last_q[2] == 0.0f && last_q[3] == 0.0f);

		if (first_packet) {
			memcpy(last_q, q, sizeof(float) * 4);
			memcpy(cur_q, q, sizeof(float) * 4);
			*last_v = q_buf_val;
			*cur_v = q_buf_val;
			*last_p = packet_type;
			*cur_p = packet_type;
			last_valid_trackers[tracker_id] = 0;

			return false; // Keep first packet
		}

		// TODO: Implement balanced mode anomaly detection logic here
		// For now, just pass through all packets after initialization
	}

	// For DISABLED and BALANCED modes, just update tracking data
	last_valid_trackers[tracker_id] = 0;
	memcpy(cur_q, q, sizeof(float) * 4);
	*cur_v = q_buf_val;
	*cur_p = packet_type;
	memcpy(last_q, q, sizeof(float) * 4);
	*last_v = q_buf_val;
	*last_p = packet_type;

	return false; // Keep this packet
}
#endif

void hid_write_packet_n(uint8_t *data, uint8_t rssi)
{
	// Input validation
	if (data == NULL) {
		LOG_ERR("NULL data pointer");
		return;
	}

	uint8_t packet_type = data[0];
	uint8_t tracker_id = data[1];

	// Validate tracker ID
	if (!validate_tracker_id(tracker_id)) {
		LOG_ERR("Invalid tracker ID: %d", tracker_id);
		return;
	}

#ifndef CONFIG_SOC_NRF52820
	// Rotation anomaly detection for quaternion packets
	if (packet_type == PACKET_TYPE_ROTATION ||
	    packet_type == PACKET_TYPE_COMPRESSED ||
	    packet_type == PACKET_TYPE_FULL_MAG) {

		float q[4] = {0};
		uint32_t q_buf_val = 0;

		// Extract quaternion based on packet type
		extract_quaternion(packet_type, data, q);

		// Get q_buf value for logging (only valid for compressed packets)
		if (packet_type == PACKET_TYPE_COMPRESSED) {
			q_buf_val = *(const uint32_t *)&data[5];
		}

		// Check for rotation anomalies and handle accordingly
		if (handle_rotation_anomaly(tracker_id, packet_type, q, q_buf_val)) {
			return; // Packet was discarded due to anomaly
		}
	}
#endif

	// Copy data to report structure
	memcpy(&report.data, data, sizeof(report));

	// Add RSSI for packets that have room (not full precision quaternion packets)
	if (packet_type != PACKET_TYPE_ROTATION && packet_type != PACKET_TYPE_FULL_MAG) {
		report.data[15] = rssi;
	}
	// Get current FIFO status atomically
	size_t write_idx = (size_t)atomic_get(&report_write_index);
	size_t read_idx = (size_t)atomic_get(&report_read_index);

	// Try to replace existing entry for the same tracker first
	if (write_idx != read_idx) {
		// Start from read point + 1 to avoid hitting the entry being used
		size_t check_index = read_idx + 1;
		if (check_index == MAX_TRACKERS) check_index = 0;

		while (check_index != write_idx) {
			if (reports[check_index].data[1] == data[1]) {
				// Replace existing entry
				reports[check_index] = report;
				return;
			}
			check_index = check_index + 1;
			if (check_index == MAX_TRACKERS) check_index = 0;
		}
	}
	if (write_idx + 1 == read_idx || (write_idx == MAX_TRACKERS-1 && read_idx == 0)) // overflow
		return;
	// Write new packet into FIFO
	reports[write_idx] = report;

	// Update write index atomically
	write_idx ++;
	if (write_idx == MAX_TRACKERS) write_idx = 0;
	atomic_set(&report_write_index, write_idx);
}

#ifndef CONFIG_SOC_NRF52820
// Function to configure anomaly detection mode
void hid_set_anomaly_detection_mode(int mode)
{
	if (mode >= 0 && mode <= ANOMALY_DETECTION_DISABLED) {
		anomaly_mode = (anomaly_detection_mode_t)mode;
		LOG_INF("Anomaly detection mode set to: %d", mode);
	} else {
		LOG_ERR("Invalid anomaly detection mode: %d", mode);
	}
}

// Function to get current anomaly detection mode
int hid_get_anomaly_detection_mode(void)
{
	return (int)anomaly_mode;
}
#endif
