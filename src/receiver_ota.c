/*
 * Receiver Self-OTA Firmware Update
 *
 * Receives firmware data via USB HID, writes to a staging area in
 * upper flash, verifies CRC32, and activates via RAM-resident flash copier.
 *
 * Protocol: reuses HID OTA report types 0xF0-0xF7 with tracker_id = 0xFE.
 * Same packet formats as tracker OTA (BEGIN, DATA, STATUS, FW_INFO, etc.).
 */

#include "receiver_ota.h"
#include "esb_ota.h"
#include "connection/esb.h"
#include "hid.h"
#include "build_defines.h"
#include "thread_priority.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_BOOTLOADER_MCUBOOT) && DT_NODE_EXISTS(DT_NODELABEL(slot1_partition))
#include <zephyr/dfu/mcuboot.h>
#define RCV_OTA_USE_MCUBOOT 1
#else
#define RCV_OTA_USE_MCUBOOT 0
#endif
#include <hal/nrf_nvmc.h>
#include <string.h>

LOG_MODULE_REGISTER(receiver_ota, LOG_LEVEL_INF);

/* ── Flash configuration ─────────────────────────────────────────── */

/* MBR occupies 0x0-0x1000 and must not be overwritten via OTA */
#define RCV_OTA_FLASH_BASE      MAX(PARTITION_NODE_OFFSET(DT_CHOSEN(zephyr_code_partition)), 0x1000)
#define RCV_OTA_FLASH_PAGE_SIZE  4096

/*
 * App partition end address (before NVS storage).
 * These preserve the existing partition layout limits.
 */
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
#define RCV_OTA_FLASH_END        0
#define RCV_OTA_BOOTLOADER_SETTINGS_ADDR  0
#elif CONFIG_SOC_NRF52840
#define RCV_OTA_FLASH_END        0xDA000  /* holyiot: 0x1000-0xDA000 (868KB) */
#define RCV_OTA_BOOTLOADER_SETTINGS_ADDR  0xFF000
#elif CONFIG_SOC_NRF52833
#define RCV_OTA_FLASH_END        0x6E000  /* foxdongle33: 0x1000-0x6E000 (436KB) */
#define RCV_OTA_BOOTLOADER_SETTINGS_ADDR  0x7F000
#else
#error "Unsupported SoC for receiver OTA"
#endif

/* Max image size: half of app region (staging needs the other half) */
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
#define RCV_OTA_MAX_IMAGE_SIZE   0
#else
#define RCV_OTA_MAX_IMAGE_SIZE   ((RCV_OTA_FLASH_END - RCV_OTA_FLASH_BASE) / 2)
#endif

/* Board target string */
#ifndef CONFIG_BOARD_TARGET
#define RCV_BOARD_TARGET_STRING CONFIG_BOARD
#else
#define RCV_BOARD_TARGET_STRING CONFIG_BOARD_TARGET
#endif

/* ── Flash writer thread ─────────────────────────────────────────── */

/*
 * Flash erase takes ~85ms per page, blocking the caller. A dedicated writer
 * and two page slots let HID intake proceed while flash operations run.
 * Intake waits up to 5 seconds when its next slot is still in use.
 */

#define RCV_OTA_WRITER_STACK_SIZE  4096

K_THREAD_STACK_DEFINE(rcv_ota_writer_stack, RCV_OTA_WRITER_STACK_SIZE);
static struct k_thread rcv_ota_writer_thread;

/* Double-buffered page write request */
struct page_write_req {
	uint8_t data[RCV_OTA_FLASH_PAGE_SIZE];
	uint32_t addr;
	uint16_t len;
};

static struct page_write_req page_write_reqs[2];

/* Per-slot semaphores: ensure a slot's data is not overwritten while
 * the writer thread is still flash-writing from it. */
static K_SEM_DEFINE(slot_sem_0, 1, 1);
static K_SEM_DEFINE(slot_sem_1, 1, 1);
static struct k_sem *slot_sems[2] = { &slot_sem_0, &slot_sem_1 };

/*
 * Command-based message queue for the writer thread.
 * Values 0-1: page write slot index. Values >= 0x80: special commands.
 */
#define WRITER_CMD_VERIFY   0xFD
#define WRITER_CMD_ACTIVATE 0xFE
#define WRITER_CMD_SHUTDOWN 0xFF

K_MSGQ_DEFINE(page_write_msgq, sizeof(uint8_t), 4, 1);

static atomic_t writer_error;
static atomic_t writer_stop_requested;
/* HID command thread owns this flag. It remains set after natural exit or a
 * stop timeout, until join proves the thread and stack can be reused. */
static bool writer_created;

/* ── Bootloader settings (Adafruit UF2 bootloader) ───────────────── */

#define BANK_VALID_APP      0x01
#define BANK_INVALID_APP    0xFF

struct bootloader_settings {
	uint16_t bank_0;
	uint16_t bank_0_crc;
	uint16_t bank_1;
	uint16_t padding;
	uint32_t bank_0_size;
	uint32_t sd_image_size;
	uint32_t bl_image_size;
	uint32_t app_image_size;
	uint32_t sd_image_start;
} __attribute__((packed));

/* ── OTA State ───────────────────────────────────────────────────── */

enum rcv_ota_state {
	RCV_OTA_IDLE,
	RCV_OTA_READY,
	RCV_OTA_RECEIVING,
	RCV_OTA_VERIFYING,
	RCV_OTA_ACTIVATE_PENDING,
	RCV_OTA_ACTIVATING,
	RCV_OTA_COMPLETE,
	RCV_OTA_ERROR,
};

static struct {
	enum rcv_ota_state state;
	uint32_t image_size;
	uint32_t image_crc32;
	uint16_t total_packets;
	uint16_t next_expected_seq;
	uint32_t bytes_written;
	uint8_t  error_code;

	/* Staging area in upper flash */
	uint32_t staging_base;
	uint32_t target_flash_base;     /* Destination base address for the new firmware */

	/* At handler boundaries a positive offset means the producer owns the slot. */
	uint16_t page_buf_offset;
	uint32_t page_buf_flash_addr;
	uint8_t  next_write_slot;

	/* Status throttle */
	int64_t  last_status_time;
} rcv_ota;

static const struct device *flash_dev;

/* Prepared bootloader settings */
static struct bootloader_settings prepared_bl_settings;
static bool bl_settings_prepared;

/* ── Forward declarations ────────────────────────────────────────── */

static void rcv_ota_send_status(void);
static void rcv_ota_send_fw_info(void);
static void rcv_ota_handle_begin(const uint8_t *data, size_t len);
static void rcv_ota_handle_data(const uint8_t *data, size_t len);
static void rcv_ota_handle_verify(void);
static void rcv_ota_handle_activate(void);
static void rcv_ota_handle_abort(void);
static int  rcv_ota_flush_page_buf(void);
static void rcv_ota_start_writer(void);
static int rcv_ota_stop_writer(void);
static void rcv_ota_do_verify(void);
static void rcv_ota_do_activate(void);
static void rcv_ota_activate_and_reset(void);

/* ── Public API ──────────────────────────────────────────────────── */

bool receiver_ota_is_active(void)
{
	return rcv_ota.state != RCV_OTA_IDLE;
}

void receiver_ota_process_hid(const uint8_t *data, size_t len)
{
	if (len < 2) {
		return;
	}

	uint8_t report_type = data[0];

	/* Initialize flash device on first use */
	if (!flash_dev) {
		flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
		if (!device_is_ready(flash_dev)) {
			LOG_ERR("RCV OTA: flash device not ready");
			return;
		}
	}

	/* A timed-out stop still owns the old session. Only retirement retries
	 * and read-only queries are allowed until the writer has been joined. */
	if (atomic_get(&writer_stop_requested) &&
	    report_type != HID_OTA_BEGIN && report_type != HID_OTA_ABORT &&
	    report_type != HID_OTA_QUERY_INFO) {
		rcv_ota_send_status();
		return;
	}

	switch (report_type) {
	case HID_OTA_QUERY_INFO:
		rcv_ota_send_fw_info();
		break;
	case HID_OTA_BEGIN: {
		bool acquired_radio = !receiver_ota_is_active();
		if (acquired_radio && esb_radio_operation_begin()) {
			LOG_WRN("Receiver OTA BEGIN deferred: radio operation busy");
			rcv_ota_send_status();
			break;
		}
		esb_auto_loss_invalidate();
		rcv_ota_handle_begin(data, len);
		if (acquired_radio) {
			esb_radio_operation_end();
		}
		break;
	}
	case HID_OTA_DATA:
		rcv_ota_handle_data(data, len);
		break;
	case HID_OTA_VERIFY:
		rcv_ota_handle_verify();
		break;
	case HID_OTA_ACTIVATE:
		rcv_ota_handle_activate();
		break;
	case HID_OTA_ABORT:
		rcv_ota_handle_abort();
		break;
	default:
		LOG_WRN("RCV OTA: unknown report type 0x%02X", report_type);
		break;
	}
}

/* ── FW_INFO ─────────────────────────────────────────────────────── */

static void rcv_ota_send_fw_info(void)
{
	LOG_INF("RCV OTA: firmware info requested");

	/* Build the 66-byte FW_INFO packet (same format as tracker) */
	uint8_t info[OTA_FW_INFO_PACKET_SIZE];
	memset(info, 0, sizeof(info));

	info[0] = ESB_OTA_FW_INFO_TYPE;
	info[1] = RECEIVER_OTA_ID;
	info[2] = FW_VERSION_MAJOR;
	info[3] = FW_VERSION_MINOR;
	info[4] = FW_VERSION_PATCH;

	/* Build datetime packed */
	uint32_t build_dt = ((uint32_t)(BUILD_YEAR - 2020) & 0x7F) << 25 |
			    ((uint32_t)BUILD_MONTH & 0x0F) << 21 |
			    ((uint32_t)BUILD_DAY & 0x1F) << 16 |
			    ((uint32_t)BUILD_HOUR & 0x1F) << 11 |
			    ((uint32_t)BUILD_MIN & 0x3F) << 5 |
			    ((uint32_t)(BUILD_SEC / 2) & 0x1F);
	sys_put_be32(build_dt, &info[5]);

	/* Firmware size from linker symbol */
	extern char _flash_used[];
	sys_put_le32((uint32_t)_flash_used, &info[9]);

	/* Bootloader type */
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
	info[13] = OTA_BOOTLOADER_MCUBOOT;
#elif CONFIG_BUILD_OUTPUT_UF2
	info[13] = OTA_BOOTLOADER_ADAFRUIT_UF2;
#elif CONFIG_BOARD_HAS_NRF5_BOOTLOADER
	info[13] = OTA_BOOTLOADER_NRF5_OPENDFU;
#else
	info[13] = OTA_BOOTLOADER_NONE;
#endif

	info[14] = OTA_PROTOCOL_VERSION;

	/* Board target string */
	strncpy((char *)&info[15], RCV_BOARD_TARGET_STRING, OTA_BOARD_TARGET_MAX - 1);

	/* Flash base address */
	sys_put_be16((uint16_t)((RCV_OTA_USE_MCUBOOT ? 0 : RCV_OTA_FLASH_BASE) >> 12),
		     &info[63]);

	info[65] = crc8_ccitt(0, info, 65);

	/* Send as 6 chunked HID sub-reports (same as tracker FW_INFO relay) */
	for (int chunk = 0; chunk < 6; chunk++) {
		uint8_t hid_report[16] = {0};
		hid_report[0] = HID_OTA_FW_INFO;
		hid_report[1] = RECEIVER_OTA_ID;
		hid_report[2] = chunk;
		size_t offset = 2 + chunk * 13;
		size_t copy_len = (offset < sizeof(info)) ?
			MIN(13, sizeof(info) - offset) : 0;
		if (copy_len > 0) {
			memcpy(&hid_report[3], &info[offset], copy_len);
		}
		hid_write_packet_n(hid_report, 0);
		k_msleep(2);
	}
}

/* ── STATUS ──────────────────────────────────────────────────────── */

static void rcv_ota_send_status(void)
{
	uint8_t status_code;
	switch (rcv_ota.state) {
	case RCV_OTA_IDLE:       status_code = OTA_STATUS_IDLE; break;
	case RCV_OTA_READY:      status_code = OTA_STATUS_READY; break;
	case RCV_OTA_RECEIVING:  status_code = OTA_STATUS_RECEIVING; break;
	case RCV_OTA_VERIFYING:  status_code = rcv_ota.error_code ?
		rcv_ota.error_code : OTA_STATUS_RECEIVING; break;
	case RCV_OTA_ACTIVATE_PENDING: status_code = OTA_STATUS_RECEIVING; break;
	case RCV_OTA_ACTIVATING: status_code = OTA_STATUS_ACTIVATING; break;
	case RCV_OTA_COMPLETE:   status_code = OTA_STATUS_COMPLETE; break;
	case RCV_OTA_ERROR:      status_code = rcv_ota.error_code; break;
	default:                 status_code = OTA_STATUS_ERROR; break;
	}

	if (atomic_get(&writer_stop_requested)) {
		status_code = OTA_STATUS_TIMEOUT;
	}

	uint8_t hid_report[16] = {0};
	hid_report[0] = HID_OTA_STATUS;
	hid_report[1] = RECEIVER_OTA_ID;
	hid_report[2] = status_code;
	sys_put_be16(rcv_ota.next_expected_seq, &hid_report[3]);
	sys_put_le32(rcv_ota.bytes_written, &hid_report[5]);
	/* hid_report[9] = ring_count (0 for receiver self-OTA, no ring buffer) */

	hid_write_packet_n(hid_report, 0);
	if (!atomic_get(&writer_stop_requested)) {
		rcv_ota.last_status_time = k_uptime_get();
	}
}

/* ── BEGIN ────────────────────────────────────────────────────────── */

static void rcv_ota_handle_begin(const uint8_t *data, size_t len)
{
	if (len < 64) {
		LOG_ERR("RCV OTA BEGIN: packet too short (%zu)", len);
		return;
	}

	/* nRF5 OpenDFU bootloader sets ACL write-protection on the app region,
	 * preventing in-place flash copy.  Reject receiver self-OTA early. */
#if CONFIG_BOARD_HAS_NRF5_BOOTLOADER && !CONFIG_BUILD_OUTPUT_UF2 && !CONFIG_BOOTLOADER_MCUBOOT
	LOG_ERR("RCV OTA: self-update blocked - nRF5 OpenDFU bootloader ACL "
		"write-protects app region. Use DFU/SWD to update this receiver.");
	rcv_ota.state = RCV_OTA_ERROR;
	rcv_ota.error_code = OTA_STATUS_ERROR;
	rcv_ota_send_status();
	return;
#endif

#if defined(CONFIG_BOOTLOADER_MCUBOOT) && !RCV_OTA_USE_MCUBOOT
	LOG_ERR("RCV OTA: MCUboot self-update requires a secondary slot");
	rcv_ota.state = RCV_OTA_ERROR;
	rcv_ota.error_code = OTA_STATUS_ERROR;
	rcv_ota_send_status();
	return;
#endif

	/* Reject duplicate if already in progress */
	if (rcv_ota.state != RCV_OTA_IDLE && rcv_ota.state != RCV_OTA_ERROR &&
	    rcv_ota.state != RCV_OTA_COMPLETE && !atomic_get(&writer_stop_requested)) {
		LOG_WRN("RCV OTA BEGIN: session active (state=%d), sending status", rcv_ota.state);
		rcv_ota_send_status();
		return;
	}

	/* Retire the old owner before even validation can change session state,
	 * and before resetting buffers or erasing MCUboot staging flash. */
	if (rcv_ota_stop_writer()) {
		rcv_ota_send_status();
		return;
	}

	/* Parse parameters (same format as tracker BEGIN) */
	uint32_t image_size = sys_get_le32(&data[2]);
	uint32_t image_crc32 = sys_get_le32(&data[6]);
	uint16_t total_packets = sys_get_be16(&data[10]);
	uint8_t protocol_ver = data[12];
	char board_target[OTA_BOARD_TARGET_MAX];
	memcpy(board_target, &data[13], OTA_BOARD_TARGET_MAX - 1);
	board_target[OTA_BOARD_TARGET_MAX - 1] = '\0';
	uint32_t flash_base = (uint32_t)sys_get_be16(&data[61]) << 12;

	LOG_INF("RCV OTA BEGIN: size=%u, crc32=0x%08X, packets=%u, proto=%u, board=%s, base=0x%X",
		image_size, image_crc32, total_packets, protocol_ver, board_target, flash_base);

	/* Validate protocol version */
	if (protocol_ver != OTA_PROTOCOL_VERSION) {
		LOG_ERR("RCV OTA: unsupported protocol version %u", protocol_ver);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_ERROR;
		rcv_ota_send_status();
		return;
	}

	/* Validate image size */
#if RCV_OTA_USE_MCUBOOT
	if (!boot_is_img_confirmed()) {
		LOG_ERR("RCV OTA: current MCUboot test image is not confirmed");
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_ERROR;
		rcv_ota_send_status();
		return;
	}
	const struct flash_area *secondary;
	int area_err = flash_area_open(PARTITION_ID(slot1_partition), &secondary);
	size_t image_offset = area_err ? 0 :
		boot_get_image_start_offset(PARTITION_ID(slot1_partition));
	ssize_t trailer_offset = area_err ? area_err :
		boot_get_area_trailer_status_offset(PARTITION_ID(slot1_partition));
	uint32_t mcuboot_capacity = trailer_offset < 0 || (size_t)trailer_offset <= image_offset ?
		0 : (uint32_t)trailer_offset - image_offset;
	if (area_err || trailer_offset < 0 || (size_t)trailer_offset <= image_offset ||
	    image_size == 0 || image_size > mcuboot_capacity) {
		if (!area_err) {
			flash_area_close(secondary);
		}
		LOG_ERR("RCV OTA: invalid MCUboot image size %u (capacity %d, err %d)",
			image_size, (int)mcuboot_capacity, area_err);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SIZE_ERROR;
		rcv_ota_send_status();
		return;
	}
#else
	if (image_size == 0 || image_size > RCV_OTA_MAX_IMAGE_SIZE) {
		LOG_ERR("RCV OTA: invalid image size %u (max %u)", image_size, RCV_OTA_MAX_IMAGE_SIZE);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SIZE_ERROR;
		rcv_ota_send_status();
		return;
	}
#endif

	/* Validate board target */
	const char *my_board = RCV_BOARD_TARGET_STRING;
	if (strncmp(board_target, my_board, OTA_BOARD_TARGET_MAX) != 0) {
#if RCV_OTA_USE_MCUBOOT
		flash_area_close(secondary);
#endif
		LOG_ERR("RCV OTA: board mismatch (got '%s', expected '%s')", board_target, my_board);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_BOARD_MISMATCH;
		rcv_ota_send_status();
		return;
	}

	/* Validate flash base — allow lower or equal base (e.g., SoftDevice → no-SD).
	 * Block higher base: target firmware expects SoftDevice not present. */
#if RCV_OTA_USE_MCUBOOT
	if (flash_base != 0) {
		flash_area_close(secondary);
		LOG_ERR("RCV OTA: MCUboot update image must use flash base 0");
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SIZE_ERROR;
		rcv_ota_send_status();
		return;
	}
#else
	if (flash_base != 0 && flash_base < 0x1000) {
		LOG_ERR("RCV OTA: flash base 0x%X below MBR (minimum 0x1000)", flash_base);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SIZE_ERROR;
		rcv_ota_send_status();
		return;
	}
#endif
	if (flash_base != 0 && flash_base > RCV_OTA_FLASH_BASE) {
		LOG_ERR("RCV OTA: flash base 0x%X > running base 0x%X - "
			"target firmware requires SoftDevice not present",
			flash_base, RCV_OTA_FLASH_BASE);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SIZE_ERROR;
		rcv_ota_send_status();
		return;
	}
	if (flash_base != 0 && (flash_base + image_size) > RCV_OTA_FLASH_END) {
		LOG_ERR("RCV OTA: image at 0x%X + %u exceeds flash end 0x%X",
			flash_base, image_size, RCV_OTA_FLASH_END);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SIZE_ERROR;
		rcv_ota_send_status();
		return;
	}

	/* Calculate staging area (page-aligned, at end of app partition) */
#if RCV_OTA_USE_MCUBOOT
	uint32_t staging_base = secondary->fa_off + image_offset;
	uint32_t secondary_size = secondary->fa_size;
	flash_area_close(secondary);
#else
	uint32_t image_pages = (image_size + RCV_OTA_FLASH_PAGE_SIZE - 1) / RCV_OTA_FLASH_PAGE_SIZE;
	uint32_t staging_base = RCV_OTA_FLASH_END - (image_pages * RCV_OTA_FLASH_PAGE_SIZE);
	staging_base &= ~(RCV_OTA_FLASH_PAGE_SIZE - 1);

	/* Verify staging doesn't overlap running firmware (or final image region) */
	extern char _flash_used[];
	uint32_t current_app_size = (uint32_t)_flash_used;
	if (staging_base < RCV_OTA_FLASH_BASE + MAX(image_size, current_app_size)) {
		LOG_ERR("RCV OTA: image too large for staging (%u bytes, staging at 0x%05X)",
			image_size, staging_base);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SIZE_ERROR;
		rcv_ota_send_status();
		return;
	}
#endif

	/* Initialize state */
	memset(&rcv_ota, 0, sizeof(rcv_ota));
	rcv_ota.image_size = image_size;
	rcv_ota.image_crc32 = image_crc32;
	rcv_ota.total_packets = total_packets;
	rcv_ota.staging_base = staging_base;
	rcv_ota.page_buf_flash_addr = staging_base;
	rcv_ota.target_flash_base = RCV_OTA_USE_MCUBOOT ? 0 :
		((flash_base != 0) ? flash_base : RCV_OTA_FLASH_BASE);
	rcv_ota.state = RCV_OTA_READY;

#if !RCV_OTA_USE_MCUBOOT
	if (rcv_ota.target_flash_base != RCV_OTA_FLASH_BASE) {
		LOG_WRN("RCV OTA: Cross-base update: running at 0x%X, target at 0x%X",
			RCV_OTA_FLASH_BASE, rcv_ota.target_flash_base);
	}
#endif

#if RCV_OTA_USE_MCUBOOT
	int erase_err = flash_flatten(flash_dev, staging_base, secondary_size);
	if (erase_err) {
		LOG_ERR("RCV OTA: failed to erase MCUboot secondary slot: %d", erase_err);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
		rcv_ota_send_status();
		return;
	}
	LOG_INF("RCV OTA: MCUboot update image at 0x%05X", staging_base);
#else
	LOG_INF("RCV OTA: staging at 0x%05X (%u pages)", staging_base, image_pages);
#endif

	bl_settings_prepared = false;

	/* Start background flash writer thread */
	rcv_ota_start_writer();

	rcv_ota_send_status();
}

/* ── DATA ────────────────────────────────────────────────────────── */

static void rcv_ota_handle_data(const uint8_t *data, size_t len)
{
	if (rcv_ota.state != RCV_OTA_READY && rcv_ota.state != RCV_OTA_RECEIVING) {
		return;
	}

	/* Check for background writer errors */
	if (atomic_get(&writer_error)) {
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
		rcv_ota_send_status();
		return;
	}

	if (len < 4) {
		return;
	}

	uint16_t seq = sys_get_be16(&data[2]);

	/* Calculate expected payload for this seq */
	uint32_t offset = (uint32_t)seq * OTA_DATA_MAX_PAYLOAD;
	if (offset >= rcv_ota.image_size) {
		LOG_WRN("RCV OTA: seq %u beyond image end", seq);
		return;
	}
	size_t payload_len = MIN(OTA_DATA_MAX_PAYLOAD, rcv_ota.image_size - offset);

	/* Sequence check */
	if (seq != rcv_ota.next_expected_seq) {
		if (seq < rcv_ota.next_expected_seq) {
			/* Duplicate, ignore */
			return;
		}
		LOG_WRN("RCV OTA: seq gap (expected %u, got %u)", rcv_ota.next_expected_seq, seq);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SEQ_ERROR;
		rcv_ota_send_status();
		return;
	}
	if (len - 4 < payload_len) {
		return;
	}

	rcv_ota.state = RCV_OTA_RECEIVING;

	/* Accumulate data into page buffer */
	const uint8_t *payload = &data[4];
	if (rcv_ota.bytes_written == 0 && payload_len >= sizeof(uint32_t)) {
		bool mcuboot_image = sys_get_le32(payload) == 0x96F3B83D;
#if RCV_OTA_USE_MCUBOOT
		if (!mcuboot_image) {
			LOG_ERR("RCV OTA: MCUboot update image header is missing");
			rcv_ota.state = RCV_OTA_ERROR;
			rcv_ota.error_code = OTA_STATUS_VERIFY_FAIL;
			rcv_ota_send_status();
			return;
		}
#else
		if (mcuboot_image) {
			LOG_ERR("RCV OTA: MCUboot image is incompatible with this bootloader");
			rcv_ota.state = RCV_OTA_ERROR;
			rcv_ota.error_code = OTA_STATUS_VERIFY_FAIL;
			rcv_ota_send_status();
			return;
		}
#endif
	}
	size_t copied = 0;

	while (copied < payload_len) {
		size_t space = RCV_OTA_FLASH_PAGE_SIZE - rcv_ota.page_buf_offset;
		size_t chunk = MIN(space, payload_len - copied);
		uint8_t slot = rcv_ota.next_write_slot;

		if (rcv_ota.page_buf_offset == 0 &&
		    k_sem_take(slot_sems[slot], K_MSEC(5000)) != 0) {
			LOG_ERR("RCV OTA: slot %u busy (timeout)", slot);
			rcv_ota.state = RCV_OTA_ERROR;
			rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
			rcv_ota_send_status();
			return;
		}

		memcpy(&page_write_reqs[slot].data[rcv_ota.page_buf_offset], &payload[copied], chunk);
		rcv_ota.page_buf_offset += chunk;
		copied += chunk;

		/* Flush full page to staging flash */
		if (rcv_ota.page_buf_offset >= RCV_OTA_FLASH_PAGE_SIZE) {
			int err = rcv_ota_flush_page_buf();
			if (err) {
				rcv_ota.state = RCV_OTA_ERROR;
				rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
				rcv_ota_send_status();
				return;
			}
		}
	}

	rcv_ota.bytes_written += payload_len;
	rcv_ota.next_expected_seq = seq + 1;

	/* Periodic status (every 20ms during receiving) */
	int64_t now = k_uptime_get();
	if (now - rcv_ota.last_status_time >= 20) {
		rcv_ota_send_status();
	}
}

/* ── VERIFY ──────────────────────────────────────────────────────── */

static void rcv_ota_handle_verify(void)
{
	if (rcv_ota.state != RCV_OTA_RECEIVING && rcv_ota.state != RCV_OTA_READY) {
		rcv_ota_send_status();
		return;
	}
	if (rcv_ota.bytes_written != rcv_ota.image_size) {
		LOG_ERR("RCV OTA: image incomplete (%u/%u bytes)",
			rcv_ota.bytes_written, rcv_ota.image_size);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_SEQ_ERROR;
		rcv_ota_send_status();
		return;
	}

	LOG_INF("RCV OTA: verify requested");
	rcv_ota.state = RCV_OTA_VERIFYING;

	/* Flush remaining data in page buffer */
	if (rcv_ota.page_buf_offset > 0) {
		int err = rcv_ota_flush_page_buf();
		if (err) {
			rcv_ota.state = RCV_OTA_ERROR;
			rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
			rcv_ota_send_status();
			return;
		}
	}

	/* Submit verify command to writer thread (processed after all pages) */
	uint8_t cmd = WRITER_CMD_VERIFY;
	if (k_msgq_put(&page_write_msgq, &cmd, K_MSEC(5000)) != 0) {
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_ERROR;
		rcv_ota_send_status();
	}
	/* Writer thread will send STATUS when done */
}

/* ── ACTIVATE ────────────────────────────────────────────────────── */

static void rcv_ota_handle_activate(void)
{
	if (rcv_ota.state != RCV_OTA_VERIFYING ||
	    rcv_ota.error_code != OTA_STATUS_VERIFY_OK) {
		LOG_ERR("RCV OTA: cannot activate without verify OK");
		rcv_ota_send_status();
		return;
	}

	LOG_WRN("RCV OTA: activate requested");
	rcv_ota.state = RCV_OTA_ACTIVATE_PENDING;

	/* Submit activate command to writer thread */
	uint8_t cmd = WRITER_CMD_ACTIVATE;
	if (k_msgq_put(&page_write_msgq, &cmd, K_MSEC(5000)) != 0) {
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_ERROR;
		rcv_ota_send_status();
	}
	/* Writer thread will do CRC16 + flash copy + reset */
}

/* ── ABORT ────────────────────────────────────────────────────────── */

static void rcv_ota_handle_abort(void)
{
	LOG_INF("RCV OTA: session aborted");
	if (rcv_ota_stop_writer()) {
		rcv_ota_send_status();
		return;
	}
	memset(&rcv_ota, 0, sizeof(rcv_ota));
	bl_settings_prepared = false;
}

/* ── Flash helpers ───────────────────────────────────────────────── */

/* Writer thread: dequeues page write requests and commands, performs flash ops */
static void rcv_ota_writer_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	while (!atomic_get(&writer_stop_requested)) {
		uint8_t cmd;
		int ret = k_msgq_get(&page_write_msgq, &cmd, K_FOREVER);
		if (ret != 0 || atomic_get(&writer_stop_requested)) {
			break;
		}

		if (cmd == WRITER_CMD_SHUTDOWN) {
			break;
		}

		if (cmd == WRITER_CMD_VERIFY) {
			rcv_ota_do_verify();
			continue;
		}

		if (cmd == WRITER_CMD_ACTIVATE) {
			rcv_ota_do_activate();
			/* Activation errors can return; the creator must still join us. */
			break;
		}

		/* cmd = 0 or 1 → page write slot */
		if (cmd > 1) {
			LOG_ERR("RCV OTA writer: unknown cmd %u", cmd);
			continue;
		}

		struct page_write_req *req = &page_write_reqs[cmd];
		uint32_t addr = req->addr;
		int err;

#if !RCV_OTA_USE_MCUBOOT
		err = flash_erase(flash_dev, addr, RCV_OTA_FLASH_PAGE_SIZE);
		if (err) {
			LOG_ERR("RCV OTA: flash erase failed at 0x%05X (err %d)", addr, err);
			atomic_set(&writer_error, 1);
			k_sem_give(slot_sems[cmd]);
			continue;
		}
#endif

		/* Write data */
		err = flash_write(flash_dev, addr, req->data, req->len);
		if (err) {
			LOG_ERR("RCV OTA: flash write failed at 0x%05X (err %d)", addr, err);
			atomic_set(&writer_error, 1);
			k_sem_give(slot_sems[cmd]);
			continue;
		}

		LOG_DBG("RCV OTA: wrote %u bytes to flash 0x%05X", req->len, addr);

		/* Release slot only after the final access to its request. */
		k_sem_give(slot_sems[cmd]);
	}
}

static void rcv_ota_start_writer(void)
{
	/* BEGIN has already joined the old thread before changing its session. */
	__ASSERT_NO_MSG(!writer_created);
	atomic_clear(&writer_error);
	atomic_clear(&writer_stop_requested);
	k_msgq_purge(&page_write_msgq);
	/* Reset slot semaphores to available */
	k_sem_reset(&slot_sem_0);
	k_sem_give(&slot_sem_0);
	k_sem_reset(&slot_sem_1);
	k_sem_give(&slot_sem_1);

	writer_created = true;
	k_thread_create(&rcv_ota_writer_thread, rcv_ota_writer_stack,
			RCV_OTA_WRITER_STACK_SIZE,
			rcv_ota_writer_fn, NULL, NULL, NULL,
			RCV_OTA_WRITER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&rcv_ota_writer_thread, "ota_writer");
}

static int rcv_ota_stop_writer(void)
{
	if (!writer_created) {
		return 0;
	}

	atomic_set(&writer_stop_requested, 1);
	/* Wake an idle writer. A full queue already guarantees a wakeup. An
	 * admitted flash operation finishes normally before the thread exits. */
	uint8_t cmd = WRITER_CMD_SHUTDOWN;
	k_msgq_put(&page_write_msgq, &cmd, K_NO_WAIT);
	int err = k_thread_join(&rcv_ota_writer_thread, K_MSEC(5000));
	if (err) {
		LOG_ERR("RCV OTA: writer stop failed (%d); session retained", err);
		return err;
	}

	/* An unsubmitted page remains producer-owned until the writer is joined. */
	if (rcv_ota.page_buf_offset > 0) {
		k_sem_give(slot_sems[rcv_ota.next_write_slot]);
		rcv_ota.page_buf_offset = 0;
	}

	writer_created = false;
	atomic_clear(&writer_stop_requested);
	return 0;
}

/*
 * CRC32 verification — runs in writer thread context (not on system workqueue).
 * This avoids blocking USB while reading entire staging area.
 */
static void rcv_ota_do_verify(void)
{
	LOG_INF("RCV OTA: verifying CRC32...");

	if (atomic_get(&writer_error)) {
		LOG_ERR("RCV OTA: flash writer had errors, skipping verify");
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
		rcv_ota_send_status();
		return;
	}

	/* Compute CRC32 over staging area */
	uint32_t crc = 0;
	uint8_t buf[256];
	uint32_t addr = rcv_ota.staging_base;
	uint32_t remaining = rcv_ota.image_size;

	while (remaining > 0) {
		size_t chunk = MIN(remaining, sizeof(buf));
		int err = flash_read(flash_dev, addr, buf, chunk);
		if (err) {
			LOG_ERR("RCV OTA: flash read error at 0x%05X: %d", addr, err);
			rcv_ota.state = RCV_OTA_ERROR;
			rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
			rcv_ota_send_status();
			return;
		}
		crc = crc32_ieee_update(crc, buf, chunk);
		addr += chunk;
		remaining -= chunk;
	}

	if (crc != rcv_ota.image_crc32) {
		LOG_ERR("RCV OTA: CRC32 mismatch (got 0x%08X, expected 0x%08X)",
			crc, rcv_ota.image_crc32);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_VERIFY_FAIL;
		rcv_ota_send_status();
		return;
	}

	LOG_INF("RCV OTA: CRC32 verified OK");
	rcv_ota.error_code = OTA_STATUS_VERIFY_OK;
	rcv_ota_send_status();
}

/*
 * Activation — runs in writer thread context.
 * Prepares bootloader settings, then calls RAM copier (never returns).
 */
static void rcv_ota_do_activate(void)
{
	LOG_WRN("RCV OTA: activating new firmware...");
	rcv_ota.state = RCV_OTA_ACTIVATING;
	rcv_ota_send_status();

#if RCV_OTA_USE_MCUBOOT
	int err = boot_request_upgrade(BOOT_UPGRADE_TEST);
	if (err) {
		LOG_ERR("RCV OTA: failed to request MCUboot test upgrade: %d", err);
		rcv_ota.state = RCV_OTA_ERROR;
		rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
		rcv_ota_send_status();
		return;
	}
	rcv_ota.state = RCV_OTA_COMPLETE;
	rcv_ota_send_status();
	k_msleep(100);
	sys_reboot(SYS_REBOOT_COLD);
	return;
#endif

	/* Prepare bootloader settings */
#if (CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER) && !CONFIG_BOOTLOADER_MCUBOOT
	/* Bootloader needs settings page update to validate the new app */
	uint16_t crc16 = 0xFFFF;
	uint8_t buf[256];
	uint32_t addr = rcv_ota.staging_base;
	uint32_t remaining = rcv_ota.image_size;
	while (remaining > 0) {
		size_t chunk = MIN(remaining, sizeof(buf));
		int err = flash_read(flash_dev, addr, buf, chunk);
		if (err) {
			LOG_ERR("RCV OTA: flash read error at 0x%05X: %d", addr, err);
			rcv_ota.state = RCV_OTA_ERROR;
			rcv_ota.error_code = OTA_STATUS_FLASH_ERROR;
			rcv_ota_send_status();
			return;
		}
		/* Nordic SDK CRC-16 (same as nRF5 bootloader / Adafruit UF2) */
		for (size_t i = 0; i < chunk; i++) {
			crc16 = (uint8_t)(crc16 >> 8) | (crc16 << 8);
			crc16 ^= buf[i];
			crc16 ^= (uint8_t)(crc16 & 0xFF) >> 4;
			crc16 ^= (crc16 << 8) << 4;
			crc16 ^= ((crc16 & 0xFF) << 4) << 1;
		}
		addr += chunk;
		remaining -= chunk;
	}

	prepared_bl_settings = (struct bootloader_settings){
		.bank_0 = BANK_VALID_APP,
		.bank_0_crc = crc16,
		.bank_1 = BANK_INVALID_APP,
		.padding = 0,
		.bank_0_size = rcv_ota.image_size,
	};
	bl_settings_prepared = true;
	LOG_INF("RCV OTA: bootloader settings prepared (CRC16=0x%04X)", crc16);
#else
	/* No bootloader — no settings page needed */
	bl_settings_prepared = false;
#endif

	k_msleep(100); /* Let status + log flush */

	rcv_ota_activate_and_reset();
	/* Never reached */
}

static int rcv_ota_flush_page_buf(void)
{
	if (atomic_get(&writer_error)) {
		return -EIO;
	}

	/* The producer already owns this slot while the page is nonempty. */
	uint8_t slot = rcv_ota.next_write_slot;
	struct page_write_req *req = &page_write_reqs[slot];

	/* Fill remainder with 0xFF */
	if (rcv_ota.page_buf_offset < RCV_OTA_FLASH_PAGE_SIZE) {
		memset(&req->data[rcv_ota.page_buf_offset], 0xFF,
		       RCV_OTA_FLASH_PAGE_SIZE - rcv_ota.page_buf_offset);
	}
	req->addr = rcv_ota.page_buf_flash_addr;
	req->len = rcv_ota.page_buf_offset;

	/* Successful submission transfers ownership to the writer. */
	int ret = k_msgq_put(&page_write_msgq, &slot, K_MSEC(5000));
	if (ret != 0) {
		LOG_ERR("RCV OTA: page write queue full (timeout)");
		/* The producer retains the unsubmitted page until retirement. */
		return -ENOSPC;
	}

	rcv_ota.next_write_slot = 1 - slot;
	rcv_ota.page_buf_flash_addr += RCV_OTA_FLASH_PAGE_SIZE;
	rcv_ota.page_buf_offset = 0;

	return 0;
}

/* ── RAM-resident flash copier ───────────────────────────────────── */

/*
 * Copies staging area to app region with IRQs disabled.
 * Same approach as tracker OTA: copy function to RAM, disable IRQs + MPU, execute.
 */

struct flash_copy_params {
	uint32_t src_addr;
	uint32_t dst_addr;
	uint32_t size;
	uint32_t page_size;
	uint32_t settings_addr;
	uint32_t settings_data[8];
	uint32_t settings_words;
};

__attribute__((noinline))
static void rcv_ota_flash_copy_from_ram(const struct flash_copy_params *p)
{
	uint32_t pages = (p->size + p->page_size - 1) / p->page_size;

	for (uint32_t i = 0; i < pages; i++) {
		uint32_t dst_page = p->dst_addr + i * p->page_size;
		uint32_t src_page = p->src_addr + i * p->page_size;

		/* Feed hardware WDT */
		((volatile uint32_t *)0x40010600)[0] = 0x6E524635;

		uint32_t remaining = p->size - i * p->page_size;
		uint32_t words = p->page_size / 4;
		if (remaining < p->page_size) {
			words = (remaining + 3) / 4;
		}

		/* Compare — skip identical pages */
		const volatile uint32_t *cmp_dst = (const volatile uint32_t *)dst_page;
		const volatile uint32_t *cmp_src = (const volatile uint32_t *)src_page;
		bool page_match = true;
		for (uint32_t w = 0; w < words; w++) {
			if (cmp_dst[w] != cmp_src[w]) {
				page_match = false;
				break;
			}
		}
		if (page_match) {
			continue;
		}

		/* Erase destination page */
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
		__DSB();
		NRF_NVMC->ERASEPAGE = dst_page;
		while (!NRF_NVMC->READY) {}

		/* Write from staging */
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		__DSB();

		volatile uint32_t *dst = (volatile uint32_t *)dst_page;
		const volatile uint32_t *src = (const volatile uint32_t *)src_page;

		for (uint32_t w = 0; w < words; w++) {
			dst[w] = src[w];
			while (!NRF_NVMC->READY) {}
		}
	}

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	__DSB();

	/* Write bootloader settings if requested */
	if (p->settings_addr != 0) {
		((volatile uint32_t *)0x40010600)[0] = 0x6E524635;

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
		__DSB();
		NRF_NVMC->ERASEPAGE = p->settings_addr;
		while (!NRF_NVMC->READY) {}

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		__DSB();
		volatile uint32_t *dst = (volatile uint32_t *)p->settings_addr;
		for (uint32_t w = 0; w < p->settings_words; w++) {
			dst[w] = p->settings_data[w];
			while (!NRF_NVMC->READY) {}
		}

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
		__DSB();
	}

	/* Reset */
	SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
	__DSB();
	for (;;) {}
}

static void rcv_ota_activate_and_reset(void)
{
	LOG_WRN("RCV OTA: Copying %u bytes from staging 0x%05X to final 0x%05X",
		rcv_ota.image_size, rcv_ota.staging_base, rcv_ota.target_flash_base);

	/* Diagnostic: dump ACL regions to identify write-protected areas */
	volatile uint32_t *acl_base = (volatile uint32_t *)0x4001E800;
	for (int i = 0; i < 8; i++) {
		uint32_t addr = acl_base[i * 4];
		uint32_t size = acl_base[i * 4 + 1];
		uint32_t perm = acl_base[i * 4 + 2];
		if (size > 0) {
			LOG_WRN("ACL[%d]: addr=0x%08X size=0x%X perm=0x%X", i, addr, size, perm);
		}
	}

	k_msleep(200);

	/* Copy flash copier function to RAM */
	static uint8_t __aligned(4) ram_func_buf[768];
	uintptr_t func_addr = (uintptr_t)rcv_ota_flash_copy_from_ram;
	uintptr_t func_start = func_addr & ~1U;
	memcpy(ram_func_buf, (void *)func_start, sizeof(ram_func_buf));

	static struct flash_copy_params params;
	params.src_addr = rcv_ota.staging_base;
	params.dst_addr = rcv_ota.target_flash_base;
	params.size = rcv_ota.image_size;
	params.page_size = RCV_OTA_FLASH_PAGE_SIZE;

	if (bl_settings_prepared) {
		params.settings_addr = RCV_OTA_BOOTLOADER_SETTINGS_ADDR;
		memcpy(params.settings_data, &prepared_bl_settings,
		       sizeof(prepared_bl_settings));
		params.settings_words = sizeof(prepared_bl_settings) / 4;
	} else {
		params.settings_addr = 0;
		params.settings_words = 0;
	}

	typedef void (*flash_copy_fn)(const struct flash_copy_params *);
	flash_copy_fn ram_copy = (flash_copy_fn)((uintptr_t)ram_func_buf | 1U);

	__disable_irq();

	MPU->CTRL = 0;
	__DSB();
	__ISB();

	ram_copy(&params);
	/* Never reached */
}
