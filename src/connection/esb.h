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
/* PING data[7]: low seven bits acknowledge a command; bit 7 requests a
 * dedicated physical-channel response, without changing legacy NORMAL. */
#define ESB_PING_FLAG_CHANNEL_CONFIRM 0x80
#define ESB_PING_COMMAND_MASK 0x7F
/* Response only: byte 8 is physical channel, byte 9 version, bytes 10-11 zero. */
#define ESB_PONG_FLAG_CHANNEL_CONFIRM 0x37
#define ESB_CHANNEL_CONFIRM_VERSION 1
#define ESB_MAX_PAYLOAD_LEN CONFIG_ESB_MAX_PAYLOAD_LENGTH
#define ESB_COMPOSITE_TYPE 0xFE // Composite packet containing multiple sub-packets

// Reference table for channel picking (see esb.c for the array itself).
bool esb_channel_is_allowed(uint8_t channel);

/* Deliberate long RX blackout boundary only. Caller owns radio and has
 * quiesced reception. No pre-boundary FIFO data may be delivered: pending
 * entries must remain inaccessible and be discarded before RX resumes.
 * Reanchors ordinary packets and invalidates loss windows, preserving totals
 * and the TDMA cap. Never call for short home beacons/automatic scan steps. */
void esb_rx_discontinuity(void);

/* ---------------------------------------------------------------------------
 * RF channel storage encoding (NVS byte). User-facing channel is 0-100.
 *   0xFF -> default channel;  0 -> invalid/legacy -> default;
 *   128  -> channel 0;        1..100 -> channel value.
 * Keeps 0 unambiguous as "no setting" across firmware upgrades. */
#define ESB_RF_CHANNEL_DEFAULT 0xFF
#define ESB_RF_CHANNEL_ZERO_ENC 128

static inline uint8_t esb_rf_channel_encode(uint8_t channel)
{
	return channel == 0 ? ESB_RF_CHANNEL_ZERO_ENC : channel;
}

static inline uint8_t esb_rf_channel_decode(uint8_t stored)
{
	if (stored == ESB_RF_CHANNEL_ZERO_ENC) {
		return 0; /* channel 0 */
	}
	if (stored == ESB_RF_CHANNEL_DEFAULT || stored == 0 || stored > 100) {
		return ESB_RF_CHANNEL_DEFAULT; /* default (incl. legacy/invalid) */
	}
	return stored;
}
// Remote command flags for PONG data[7]
#define ESB_PONG_FLAG_NORMAL 0x00
#define ESB_PONG_FLAG_SHUTDOWN 0x01
#define ESB_PONG_FLAG_CALIBRATE 0x02     // Trigger gyro/accel ZRO calibration
#define ESB_PONG_FLAG_CALIBRATE_ACC 0x03 // Trigger 18-orientation accelerometer calibration
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
#define ESB_PONG_FLAG_TCAL_AUTO_ON 0x13     // Enable T-Cal auto-calibration
#define ESB_PONG_FLAG_TCAL_AUTO_OFF 0x14    // Disable T-Cal auto-calibration
#define ESB_PONG_FLAG_FUSION_RESET 0x15     // Reset fusion (invalidate quaternion)
#define ESB_PONG_FLAG_TCAL_BOOT_ON 0x16     // Enable T-Cal boot calibration
#define ESB_PONG_FLAG_TCAL_BOOT_OFF 0x17    // Disable T-Cal boot calibration
#define ESB_PONG_FLAG_MAG_CAL 0x18          // Trigger magnetometer calibration
#define ESB_PONG_FLAG_MAG_ON 0x19           // Enable magnetometer
#define ESB_PONG_FLAG_MAG_OFF 0x1A          // Disable magnetometer
#define ESB_PONG_FLAG_TCAL_ON 0x1B          // Enable T-Cal (temperature calibration)
#define ESB_PONG_FLAG_TCAL_OFF 0x1C         // Disable T-Cal (temperature calibration)
#define ESB_PONG_FLAG_TDMA_ON 0x1D          // Enable TDMA scheduling
#define ESB_PONG_FLAG_TDMA_OFF 0x1E         // Disable TDMA scheduling
#define ESB_PONG_FLAG_TEST_MODE_ON 0x1F     // Enable battery drain test mode
#define ESB_PONG_FLAG_TEST_MODE_OFF 0x20    // Disable battery drain test mode
#define ESB_PONG_FLAG_DFU_OTA 0x21          // Enter OTA DFU bootloader
#define ESB_PONG_FLAG_DATA_COLLECT_ON 0x22  // Start raw data collection
#define ESB_PONG_FLAG_DATA_COLLECT_OFF 0x23 // Stop raw data collection
#define ESB_PONG_FLAG_SENS_AUTO 0x24        // Auto-calibrate gyro sensitivity
#define ESB_PONG_FLAG_WOM_CONTROL 0x28
#define ESB_PING_FLAG_WOM_RESULT 0x29
#define ESB_PING_FLAG_WOM_ERROR 0x2A
#define ESB_WOM_TIMEOUT_MAX_MINUTES 35791
#define ESB_WOM_TIMEOUT_CLEAR 0xffff
#define ESB_WOM_ENTER 0xfffe
#define ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON 0x34  // Start batch raw data collection (data[8] = target Hz)
#define ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF 0x35 // Stop batch raw data collection
#define ESB_PONG_FLAG_METADATA_REQUEST 0x36 // Request selected metadata section/chunk
#define ESB_METADATA_MASK_VALID 0x3F
#define ESB_PONG_FLAG_MAG_AUTO_ON 0x25      // Enable online magnetometer calibration
#define ESB_PONG_FLAG_MAG_AUTO_OFF 0x26     // Disable online magnetometer calibration
#define ESB_PONG_FLAG_TCAL_HEATED_START 0x27 // Request heated T-Cal with the configured default target
#define ESB_PONG_FLAG_OTA_QUERY_INFO 0x30   // Request firmware info for ESB OTA
#define ESB_PONG_FLAG_OTA_ABORT 0x31        // Abort ESB OTA update
#define ESB_PONG_FLAG_OTA_SUPPRESS 0x32     // Suppress tracker during OTA (reduce poll rate)
#define ESB_PONG_FLAG_OTA_UNSUPPRESS 0x33   // Resume normal poll rate after OTA

// Raw data collection packet types
#define ESB_RAW_IMU_TYPE 0x10      // Raw IMU data (float, with piggybacked mag)
#define ESB_RAW_MAG_TYPE 0x11      // Raw magnetometer data (float, reserved)
// Metadata (ODR, range, sensor IDs): captured once per collection session,
// sent at session start, and replayed on explicit requests.
#define ESB_RAW_META_TYPE 0x12
#define ESB_RAW_IMU_QUAT_TYPE 0x13 // Raw IMU with gyrQuat (52 bytes, packet-loss resistant)
#define ESB_RAW_CAL_TYPE 0x14      // Extended calibration metadata (sub-typed)

// ESB OTA packet types (used during firmware update over ESB)
#define ESB_OTA_DATA_TYPE 0x20     // OTA firmware data (receiver → tracker)
#define ESB_OTA_STATUS_TYPE 0x21   // OTA status report (tracker → receiver)
#define ESB_OTA_FW_INFO_TYPE 0x22  // Firmware info report (tracker → receiver)
#define ESB_OTA_BEGIN_TYPE 0x23    // Begin OTA session (receiver → tracker)
#define ESB_OTA_VERIFY_TYPE 0x24   // Request CRC verification (receiver → tracker)
#define ESB_OTA_ACTIVATE_TYPE 0x25 // Activate new firmware (receiver → tracker)

void event_handler(struct esb_evt const *event);
int clocks_start(void);
int esb_initialize(bool);
/* Internal background transactions suppress only routine initialization INFO. */
int esb_initialize_quiet(bool tx);

void esb_set_addr_unified(void);

int esb_add_pair(uint64_t addr, bool checksum);
int esb_pop_pair(void);

void esb_start_pairing(void);
void esb_start_pairing_with_count(uint8_t target_count);
void esb_finish_pair(void);
int esb_clear(void);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
void esb_print_health_snapshot(void);
#endif

void esb_reset_all_stats(void);

/* Cumulative normal/pose-composite sequence evidence, not lifetime loss or
 * status bytes. PING/raw/duplicate/out-of-order traffic adds no receipts. */
#define ESB_AUTO_LOSS_TRACKERS 16
struct esb_auto_loss_snapshot {
	struct {
		uint32_t received;
		uint32_t gaps;
		uint32_t restarts;
	} tracker[ESB_AUTO_LOSS_TRACKERS];
	uint32_t active_mask;
	uint32_t reset_epoch;
	/* Readiness means an accepted normal anchor, NOT acknowledged layout adoption. */
	uint32_t ready_mask;
	uint8_t layout_epoch;
	bool cap_pending;
	uint8_t home;
	bool eligible;
};
void esb_get_auto_loss_snapshot(struct esb_auto_loss_snapshot *out);
/* Invalidate evidence at excluded-session admission, including short sessions
 * wholly between worker samples. Not for routine HOME beacons/auto scans. */
void esb_auto_loss_invalidate(void);
/* Fence only the next decision edge per tracker; preserve decoder anchors,
 * readiness and window epoch. Call after actual RX quiescence. */
void esb_auto_loss_fence(void);
/* Establish a common decision baseline after all required streams returned.
 * The next edge of every stream is anchor-only, including an early returner
 * whose last packet predates this baseline. Snapshot and fence are atomic. */
void esb_auto_loss_rebase(struct esb_auto_loss_snapshot *out);
void esb_write_sync(uint16_t led_clock);
void esb_receive(void);

// Statistics display control
bool esb_toggle_stats_detailed(void);                   // Toggle detailed stats on/off
void esb_set_stats_detailed(uint32_t duration_seconds); // Enable for duration (0 = toggle)
bool esb_get_stats_detailed_enabled(void);              // Get current status

// Remote command API
/* WOM control: 0=query, 1..35791=set, ESB_WOM_TIMEOUT_CLEAR=clear override,
 * ESB_WOM_ENTER=request immediate WOM admission (not proof of sleep).
 * target=0xff scans active slots (~1s). One fresh nonzero token per request;
 * zero mask means no admitted target. */
uint32_t esb_send_remote_wom_command(uint8_t target_id, uint16_t minutes, uint16_t *token_out);
/* Exact transaction outcome; positive errno on failure, value=0 on successful clear,
 * value=ESB_WOM_ENTER on accepted entry, otherwise timeout minutes. */
bool esb_get_remote_wom_result(uint8_t id, uint16_t token, uint16_t *minutes, uint16_t *error);
void esb_cancel_remote_wom_command(uint32_t mask, uint16_t token);
void esb_send_remote_command(uint8_t tracker_id, uint8_t command_flag);
/* Clear only a pending OTA abort command; preserves unrelated commands. */
void esb_clear_remote_ota_abort(uint8_t tracker_id);
void esb_send_remote_command_arg(uint8_t tracker_id, uint8_t command_flag, uint8_t arg);
/* Active-scan then queue. Returns bitmask of targeted tracker ids. Blocks ~1s. */
uint32_t esb_send_remote_command_all(uint8_t command_flag);
/* Targeted test commands: publish TPS, then queue the flag. Return the
 * targeted-tracker bitmask; invalidate sticky-all state. */
uint32_t esb_send_remote_command_test_on(uint8_t tracker_id, uint16_t tps);
uint32_t esb_send_remote_command_test_off(uint8_t tracker_id);
/* Active-scan then queue. Return bitmask of targeted tracker ids. Blocks ~1s. */
uint32_t esb_send_remote_command_test_on_all(uint16_t tps);
uint32_t esb_send_remote_command_test_off_all(void);
void esb_send_remote_command_sens(uint8_t tracker_id, float x, float y, float z);
bool esb_send_remote_command_sens_auto(uint8_t tracker_id, uint8_t axis, uint16_t revolutions);
/* Active-scan then queue. Returns bitmask of targeted tracker ids. Blocks ~1s. */
uint32_t esb_send_remote_command_sens_auto_all(uint8_t axis, uint16_t revolutions);
/* Queue a metadata repair request without replacing an existing control flag.
 * mask selects metadata sections; chunk selects a dense tcal block (0..254) or all (255). */
bool esb_request_metadata(uint8_t tracker_id, uint8_t mask, uint8_t chunk);
/* Nonblocking fixed announcement then home commit; -EBUSY for radio/transfer conflicts. */
int esb_set_all_trackers_channel(uint8_t channel);
int esb_clear_all_trackers_channel(void);

/* Called once after receiver RF + persistence commit (not tracker execution proof). */
typedef void (*esb_channel_change_done_cb_t)(bool success);
void esb_set_channel_change_done_cb(esb_channel_change_done_cb_t cb);

/* Optional: called when a tracker confirms a remote PONG flag (event_handler). */
typedef void (*esb_remote_confirm_cb_t)(uint8_t tracker_id, uint8_t flag);
void esb_set_remote_confirm_cb(esb_remote_confirm_cb_t cb);

// Local receiver channel management
int esb_set_receiver_channel(uint8_t channel); // Set local home, errno on failure
int esb_clear_receiver_channel(void);         // Restore default local home
/* Nonreentrant radio gate. Begin/end do not stop/restore hardware. Active transfer
 * predicates retain exclusion after their admission owner releases this gate. */
bool esb_channel_is_busy(void);
int esb_radio_operation_begin(void);
int esb_radio_collection_begin(void); /* Admission for replacing an existing raw session */
void esb_radio_operation_end(void);
int esb_radio_operation_stop(void);
/* Owner-held checked PTX/PRX teardown. Failure latches the radio owner fault. */
int esb_radio_quiesce(void);
/* Owner-held, after operation_stop: DPL NoACK pipe 0 on current home.
 * Always quiesces PTX; caller must restore PRX before releasing the owner. */
int esb_home_beacon_send(const uint8_t frame[16]);
bool esb_pairing_is_active(void);
int esb_auto_channel_request(uint8_t channel, uint8_t expected_home, uint32_t generation);
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
