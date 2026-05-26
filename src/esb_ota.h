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
#ifndef SLIMENRF_ESB_OTA_RECEIVER_H
#define SLIMENRF_ESB_OTA_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <esb.h>

/*
 * ESB OTA Protocol Constants (shared with tracker)
 */
#define ESB_OTA_DATA_TYPE       0x20
#define ESB_OTA_STATUS_TYPE     0x21
#define ESB_OTA_FW_INFO_TYPE    0x22
#define ESB_OTA_BEGIN_TYPE      0x23
#define ESB_OTA_VERIFY_TYPE     0x24
#define ESB_OTA_ACTIVATE_TYPE   0x25

#define OTA_DATA_HEADER_SIZE    4
#define OTA_DATA_MAX_PAYLOAD    44

#define OTA_STATUS_IDLE             0x00
#define OTA_STATUS_READY            0x01
#define OTA_STATUS_RECEIVING        0x02
#define OTA_STATUS_VERIFY_OK        0x03
#define OTA_STATUS_VERIFY_FAIL      0x04
#define OTA_STATUS_ACTIVATING       0x05
#define OTA_STATUS_COMPLETE         0x06
#define OTA_STATUS_ERROR            0x10
#define OTA_STATUS_BOARD_MISMATCH   0x11
#define OTA_STATUS_FLASH_ERROR      0x12
#define OTA_STATUS_SIZE_ERROR       0x13
#define OTA_STATUS_SEQ_ERROR        0x14
#define OTA_STATUS_TIMEOUT          0x15

#define OTA_PROTOCOL_VERSION    1
#define OTA_BOARD_TARGET_MAX    32
#define OTA_BEGIN_PACKET_SIZE   48

/* HID OTA report types (PC ↔ Receiver) */
#define HID_OTA_QUERY_INFO      0xF0
#define HID_OTA_FW_INFO         0xF1
#define HID_OTA_BEGIN           0xF2
#define HID_OTA_DATA            0xF3
#define HID_OTA_STATUS          0xF4
#define HID_OTA_VERIFY          0xF5
#define HID_OTA_ACTIVATE        0xF6
#define HID_OTA_ABORT           0xF7

/*
 * Seq-indexed ring buffer for OTA data packets (HID → ESB ACK).
 * Must be power of 2. Indexed by seq number: ring[seq & MASK].
 * 256 packets × 48 bytes = 12 KB (shared by all targets).
 * Retransmission is automatic: tracker's next_seq indexes the buffer.
 */
#define OTA_TX_RING_SIZE        256
#define OTA_TX_RING_MASK        (OTA_TX_RING_SIZE - 1)

/* Maximum number of trackers updated in parallel */
#define OTA_MAX_PARALLEL        4

/**
 * Check if an OTA session is active for any tracker.
 * ISR-safe (reads volatile).
 */
bool esb_ota_relay_is_active(void);

/**
 * Check if a specific tracker is participating in the current OTA session.
 */
bool esb_ota_relay_is_target(uint8_t tracker_id);

/**
 * Get the number of active OTA targets.
 */
uint8_t esb_ota_relay_get_num_targets(void);

/**
 * Process a HID OUT report that may contain an OTA command.
 * Called from HID read work handler (thread context).
 *
 * @param data   HID report data (64 bytes)
 * @param len    Report length
 */
void esb_ota_relay_process_hid(const uint8_t *data, size_t len);

/**
 * Called from esb_ack_handler_cb (radio ISR context) when a tracker
 * in OTA mode sends a status/poll packet. Fills the ACK payload
 * with the OTA data packet matching the tracker's next_seq.
 *
 * Uses seq-indexed lookup: the tracker's next_seq from the STATUS
 * packet directly indexes the ring buffer. If the ACK is lost,
 * the tracker re-sends the same next_seq and gets automatic
 * retransmission without PC intervention.
 *
 * @param tracker_id   The tracker that sent the poll
 * @param pipe_id      ESB pipe
 * @param ack_payload  ACK payload to fill
 * @param has_ack      Set to true if ACK payload was filled
 * @param rx_data      Received packet data (STATUS), or NULL
 * @param rx_len       Length of rx_data
 */
void esb_ota_relay_fill_ack(uint8_t tracker_id, uint32_t pipe_id,
			    struct esb_payload *ack_payload, bool *has_ack,
			    const uint8_t *rx_data, uint8_t rx_len);

/**
 * Called from event_handler (EVENT IRQ context) when the receiver
 * receives an OTA status or firmware info packet from a tracker.
 * Forwards it to PC via HID IN report.
 *
 * @param data   Received ESB packet data
 * @param len    Packet length
 */
void esb_ota_relay_process_tracker_packet(const uint8_t *data, size_t len);

/**
 * Process OTA commands from the console.
 * Called from console thread.
 */
void esb_ota_relay_console_cmd(uint8_t tracker_id, const char *cmd);

#endif /* SLIMENRF_ESB_OTA_RECEIVER_H */
