#ifndef SLIMENRF_SYSTEM
#define SLIMENRF_SYSTEM

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "led.h"
#include "status.h"

#define STORED_TRACKERS 1
#define RBT_CNT_ID 2
#define STORED_ADDR_0 3
// 0-15 -> id 3-18
// 0-255 -> id 3-258

#define RF_CHANNEL 259  // NVS key for receiver RF channel
#define TRACKER_ROSTER 260 // Versioned, atomic tracker roster

uint8_t reboot_counter_read(void);

int sys_write_checked(uint16_t id, const void *data, size_t len);
void sys_read(uint16_t id, void* data, size_t len);
/* Raw NVS record length or negative errno; never clears the destination. */
int sys_read_checked(uint16_t id, void *data, size_t len);

bool button_read(void);
bool button_read_filtered(void);
void sys_request_system_reboot(void);

/* Enter bootloader DFU (UF2 default, or OTA when ota=true on Adafruit BL). */
void sys_enter_dfu(bool ota);
/* Mark next reboot to skip DFU only when no bootloader request is pending. */
void sys_skip_dfu_marker(void);
/* Acknowledge healthy uptime only for a capability-advertising bootloader,
 * without replacing pending DFU/fatal requests or legacy marker values. */
void sys_mark_boot_success(void);

#endif
