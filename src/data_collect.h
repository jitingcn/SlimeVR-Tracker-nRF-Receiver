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

/* Clears single and batch session state.
 * CDC initializes once in usb_init_thread before USB setup. HID uses SYS_INIT.
 * Opening the console does not reinitialize either backend. */
int data_collect_init(void);

/* Write an accepted raw data packet from the ESB EVENT ISR.
 * data: raw ESB payload, len: payload length, rssi: received signal strength */
void data_collect_write(const uint8_t *data, uint8_t len, uint8_t rssi);

/* Thread control: start replaces any session; stop affects only single mode.
 * The local transition publishes the corresponding remote flags atomically. */
void data_collect_start(uint8_t tracker_id);
void data_collect_stop(void);
void data_collect_forget_tracker(uint8_t tracker_id);
bool data_collect_is_active(void);
uint8_t data_collect_get_target_id(void);

/* Thread control for simultaneous collection. Stop affects only batch mode. */
void data_collect_batch_start(uint32_t mask, uint16_t rate_hz);
void data_collect_batch_stop(void);
bool data_collect_batch_is_active(void);
bool data_collect_batch_is_target(uint8_t tracker_id);

/* Check if data collection is active and this tracker is the target */
bool data_collect_is_target(uint8_t tracker_id);
#else /* !CONFIG_DATA_COLLECT */

static inline int data_collect_init(void) { return 0; }
static inline void data_collect_write(const uint8_t *data, uint8_t len, uint8_t rssi) { (void)data; (void)len; (void)rssi; }
static inline void data_collect_start(uint8_t tracker_id) { (void)tracker_id; }
static inline void data_collect_stop(void) {}
static inline void data_collect_forget_tracker(uint8_t tracker_id) { (void)tracker_id; }
static inline bool data_collect_is_active(void) { return false; }
static inline uint8_t data_collect_get_target_id(void) { return 0; }
static inline void data_collect_batch_start(uint32_t mask, uint16_t rate_hz) { (void)mask; (void)rate_hz; }
static inline void data_collect_batch_stop(void) {}
static inline bool data_collect_batch_is_active(void) { return false; }
static inline bool data_collect_batch_is_target(uint8_t tracker_id) { (void)tracker_id; return false; }
static inline bool data_collect_is_target(uint8_t tracker_id) { (void)tracker_id; return false; }
#endif /* CONFIG_DATA_COLLECT */

#endif /* SLIMENRF_DATA_COLLECT_H */
