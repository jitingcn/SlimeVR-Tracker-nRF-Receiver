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
 * After the image is verified, the engine marks bank 0 invalid in the
 * bootloader settings page, writes page 0, then marks bank 0 valid with the
 * new size and CRC16. The
 * bootloader must leave the settings page writable after starting the
 * application (Adafruit nRF52 bootloader 0.11.1 or later; 0.11.0-slimenrf
 * write-protects it).
 */

struct rcv_ota_ram_params {
	uint32_t image_size;
	uint32_t image_crc32;
	uint32_t flash_base;      /* Application start (page aligned, >= 0x1000) */
	uint32_t flash_end;       /* Exclusive application partition end (page aligned) */
	uint32_t sram_base;       /* SRAM start address (e.g. 0x20000000) */
	uint32_t sram_end;        /* Exclusive SRAM end address (e.g. 0x20020000) */
	uint32_t settings_addr;   /* Bootloader settings page; 0 = no bootloader */
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
