/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors
*/
#ifndef SLIMENRF_RECEIVER_OTA_RAM_H
#define SLIMENRF_RECEIVER_OTA_RAM_H

#include <stdbool.h>
#include <stdint.h>

/*
 * In-place receiver self-OTA over USB HID.
 *
 * After BEGIN is validated, the receiver stops Zephyr, copies a
 * position-independent engine blob from flash into a takeover SRAM arena, and
 * runs it with interrupts and MPU disabled. The engine services the already-
 * enumerated HID endpoints directly through the USBD registers and writes the
 * new image over the running application. There is no staging area, so the
 * image limit is the whole application partition instead of half of it.
 *
 * The engine never writes the bootloader settings page. When the bootloader
 * stored a CRC16 for bank 0, the engine sets two bytes of a reserved Cortex-M
 * vector slot (offset 0x1C, zero in Zephyr images) so the new image matches
 * the stored size and CRC16. This also works on bootloaders that write-protect
 * the settings page before starting the application.
 */

struct rcv_ota_ram_params {
	uint32_t image_size;
	uint32_t image_crc32;
	uint32_t flash_base;      /* Application start (page aligned, >= 0x1000) */
	uint32_t flash_end;       /* Exclusive application partition end (page aligned) */
	uint32_t sram_base;       /* SRAM start address (e.g. 0x20000000) */
	uint32_t sram_end;        /* Exclusive SRAM end address (e.g. 0x20020000) */
	uint32_t check_size;      /* Bootloader bank_0_size; 0 = no CRC16 check */
	uint16_t check_crc16;     /* Bootloader bank_0_crc */
	uint8_t in_ep;            /* HID IN endpoint number (1-7) */
	uint8_t out_ep;           /* HID OUT endpoint number (1-7) */
	bool errata_199;          /* Apply nRF52 USBD anomaly 199 workaround */
};

/*
 * Validates parameters and takes over the CPU with interrupts disabled.
 * Returns false only if pre-takeover parameter/blob checks fail (before any
 * interrupt or peripheral state is modified); otherwise never returns.
 */
bool rcv_ota_ram_engine_run(const struct rcv_ota_ram_params *params);

#endif /* SLIMENRF_RECEIVER_OTA_RAM_H */
