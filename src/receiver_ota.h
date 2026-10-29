/*
 * Receiver Self-OTA Firmware Update
 *
 * Updates the receiver's own firmware via USB HID.
 * Uses tracker_id = 0xFE to distinguish from tracker relay OTA.
 *
 * Data flow:
 *   PC → HID OUT → receiver_ota_process_hid() → staging flash → verify → RAM copy → reset
 *
 * Flash layout:
 *   App partition:  [running firmware | ... | staging area]
 *   Staging is placed at the end of the app partition, sized to fit the new image.
 *   After CRC32 verification, a RAM-resident copier overwrites the running
 *   firmware from staging, updates bootloader settings, and resets.
 */
#ifndef SLIMENRF_RECEIVER_OTA_H
#define SLIMENRF_RECEIVER_OTA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Receiver OTA uses tracker_id = 0xFE in all HID reports */
#define RECEIVER_OTA_ID  0xFE

/**
 * Check if receiver self-OTA is active.
 */
bool receiver_ota_is_active(void);

/**
 * Process a HID OUT report for receiver self-OTA.
 * Called from HID read work handler (thread context) when tracker_id == 0xFE.
 *
 * DATA carries fixed 60-byte image chunks after its 4-byte header. The final
 * chunk may contain exactly the remaining image bytes or use a padded 64-byte
 * report; padding is ignored. Short non-final chunks are not admitted.
 *
 * VERIFY and ACTIVATE duplicates report current status without enqueueing again.
 * ACTIVATE requires completed verification. Command enqueue failure reports
 * OTA_STATUS_ERROR; ACTIVATING is published only when the writer enters activation.
 *
 * @param data   HID report data (up to 64 bytes)
 * @param len    Report length
 */
void receiver_ota_process_hid(const uint8_t *data, size_t len);

#endif /* SLIMENRF_RECEIVER_OTA_H */
