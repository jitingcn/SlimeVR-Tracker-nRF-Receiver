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
#ifndef SLIMENRF_DATA_COLLECT_H
#define SLIMENRF_DATA_COLLECT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_DATA_COLLECT

/*
 * Data collection via dedicated HID endpoint (HID_1).
 *
 * Each raw ESB packet is forwarded as a single 64-byte HID input report:
 *   [0..N-1]    ESB payload (up to 48 bytes, same format as over-the-air)
 *   [N]         RSSI
 *   [N+1..N+4]  rx_ticks (BE32, Zephyr uptime ticks)
 *   [N+5..63]   zero padding
 *
 * No framing protocol needed — HID guarantees packet boundaries.
 * The PC reads from the second HID interface (vendor usage page 0xFF00).
 *
 * Payload types (byte 0):
 *   - Type 0x10 (RAW_IMU):  48 bytes, with full-precision T-Cal temp at
 *                           bytes 41-44
 *   - Type 0x11 (RAW_MAG):  up to 17 bytes
 *   - Type 0x12 (RAW_META): 48 bytes
 */

/* Initialize data collection HID endpoint */
int data_collect_init(void);

/* Write a raw data packet as a HID report.
 * Called from ESB event handler when a raw packet (0x10-0x12) arrives.
 * data: raw ESB payload, len: payload length, rssi: received signal strength */
void data_collect_write(const uint8_t *data, uint8_t len, uint8_t rssi);

/* Runtime control: start/stop data collection for a specific tracker */
void data_collect_start(uint8_t tracker_id);
void data_collect_stop(void);
bool data_collect_is_active(void);
uint8_t data_collect_get_target_id(void);

/* Check if data collection is active and this tracker is the target */
bool data_collect_is_target(uint8_t tracker_id);

#else /* !CONFIG_DATA_COLLECT */

static inline int data_collect_init(void) { return 0; }
static inline void data_collect_write(const uint8_t *data, uint8_t len, uint8_t rssi) { (void)data; (void)len; (void)rssi; }
static inline void data_collect_start(uint8_t tracker_id) { (void)tracker_id; }
static inline void data_collect_stop(void) {}
static inline bool data_collect_is_active(void) { return false; }
static inline uint8_t data_collect_get_target_id(void) { return 0; }
static inline bool data_collect_is_target(uint8_t tracker_id) { (void)tracker_id; return false; }

#endif /* CONFIG_DATA_COLLECT */

#endif /* SLIMENRF_DATA_COLLECT_H */
