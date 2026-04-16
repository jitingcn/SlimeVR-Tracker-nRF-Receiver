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
#ifndef SLIMENRF_ESB
#define SLIMENRF_ESB

#include <esb.h>
#include <stdint.h>

// Ping/Pong constants (shared protocol)
#define ESB_PING_TYPE 0xF0
#define ESB_PONG_TYPE 0xF1
#define ESB_PING_LEN 13
#define ESB_PONG_LEN 13
#define ESB_MAX_PAYLOAD_LEN CONFIG_ESB_MAX_PAYLOAD_LENGTH
#define ESB_COMPOSITE_TYPE 0x05 // Composite packet containing multiple sub-packets

// Remote command flags for PONG data[7]
#define ESB_PONG_FLAG_NORMAL 0x00
#define ESB_PONG_FLAG_SHUTDOWN 0x01
#define ESB_PONG_FLAG_CALIBRATE 0x02     // Trigger gyro/accel ZRO calibration
#define ESB_PONG_FLAG_SIX_SIDE_CAL 0x03  // Trigger 6-point accelerometer calibration
#define ESB_PONG_FLAG_MEOW 0x04          // Trigger meow output
#define ESB_PONG_FLAG_SCAN 0x05          // Trigger sensor scan
#define ESB_PONG_FLAG_MAG_CLEAR 0x06     // Clear magnetometer calibration
#define ESB_PONG_FLAG_REBOOT 0x07        // Reboot tracker
#define ESB_PONG_FLAG_CLEAR 0x08         // Clear pairing data
#define ESB_PONG_FLAG_DFU 0x09           // Enter DFU bootloader
#define ESB_PONG_FLAG_SET_CHANNEL 0x0A   // Set RF channel (data[8-11] contains channel value)
#define ESB_PONG_FLAG_CLEAR_CHANNEL 0x0B // Clear RF channel setting (restore default)
#define ESB_PONG_FLAG_SENS_SET 0x0C
#define ESB_PONG_FLAG_SENS_RESET 0x0D
#define ESB_PONG_FLAG_RESET_ZRO 0x0E
#define ESB_PONG_FLAG_RESET_ACC 0x0F
#define ESB_PONG_FLAG_RESET_BAT 0x10
#define ESB_PONG_FLAG_PING 0x11
#define ESB_PONG_FLAG_RESET_TCAL 0x12
#define ESB_PONG_FLAG_TCAL_AUTO_ON 0x13  // Enable T-Cal auto-calibration
#define ESB_PONG_FLAG_TCAL_AUTO_OFF 0x14 // Disable T-Cal auto-calibration
#define ESB_PONG_FLAG_FUSION_RESET 0x15  // Reset fusion (invalidate quaternion)
#define ESB_PONG_FLAG_TCAL_BOOT_ON 0x16  // Enable T-Cal boot calibration
#define ESB_PONG_FLAG_TCAL_BOOT_OFF 0x17 // Disable T-Cal boot calibration
#define ESB_PONG_FLAG_MAG_CAL 0x18       // Trigger magnetometer calibration
#define ESB_PONG_FLAG_MAG_ON 0x19        // Enable magnetometer
#define ESB_PONG_FLAG_MAG_OFF 0x1A       // Disable magnetometer
#define ESB_PONG_FLAG_TCAL_ON 0x1B       // Enable T-Cal (temperature calibration)
#define ESB_PONG_FLAG_TCAL_OFF 0x1C      // Disable T-Cal (temperature calibration)
#define ESB_PONG_FLAG_TDMA_ON 0x1D       // Enable TDMA scheduling
#define ESB_PONG_FLAG_TDMA_OFF 0x1E      // Disable TDMA scheduling
#define ESB_PONG_FLAG_TEST_MODE_ON 0x1F  // Enable battery drain test mode
#define ESB_PONG_FLAG_TEST_MODE_OFF 0x20 // Disable battery drain test mode
#define ESB_PONG_FLAG_DFU_OTA 0x21       // Enter OTA DFU bootloader

void event_handler(struct esb_evt const *event);
int clocks_start(void);
int esb_initialize(bool);
void esb_deinitialize(void);

void esb_set_addr_discovery(void);
void esb_set_addr_paired(void);
void esb_set_addr_unified(void);

int esb_add_pair(uint64_t addr, bool checksum);
void esb_pop_pair(void);

void esb_start_pairing(void);
void esb_start_pairing_with_count(uint8_t target_count);
void esb_reset_pair(void);
void esb_finish_pair(void);
void esb_clear(void);
void esb_reset_tracker_sequence(uint8_t tracker_id);
void esb_print_all_stats(void);
void esb_reset_all_stats(void);
void esb_write_sync(uint16_t led_clock);
void esb_receive(void);

// Statistics display control
bool esb_toggle_stats_detailed(void);           // Toggle detailed stats on/off
void esb_set_stats_detailed(uint32_t duration_seconds); // Enable for duration (0 = toggle)
bool esb_get_stats_detailed_enabled(void);      // Get current status
uint32_t esb_get_stats_detailed_remaining(void); // Get remaining time (0 if no auto-disable)

// Remote command API
void esb_send_remote_command(uint8_t tracker_id, uint8_t command_flag);
void esb_send_remote_command_all(uint8_t command_flag);
void esb_send_remote_command_sens(uint8_t tracker_id, float x, float y, float z);
void esb_set_all_trackers_channel(uint8_t channel); // Set RF channel for all trackers
void esb_clear_all_trackers_channel(void);          // Clear RF channel setting (restore default)

// Local receiver channel management
void esb_set_receiver_channel(uint8_t channel); // Set receiver RF channel only (local)
void esb_clear_receiver_channel(void);          // Clear receiver RF channel (restore default, local)
uint8_t esb_get_receiver_channel(void);         // Get current receiver RF channel (returns 0xFF if default)

// Convenience wrappers for specific commands
static inline void esb_request_tracker_shutdown(uint8_t tracker_id)
{
	esb_send_remote_command(tracker_id, ESB_PONG_FLAG_SHUTDOWN);
}

static inline void esb_request_all_shutdown(void)
{
	esb_send_remote_command_all(ESB_PONG_FLAG_SHUTDOWN);
}

#endif
