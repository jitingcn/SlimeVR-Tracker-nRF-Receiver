#ifndef TRACKER_EVENTS_H
#define TRACKER_EVENTS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool tracker_events_receive(const uint8_t *packet, size_t len, uint32_t now_ms);
void tracker_events_process(uint32_t now_ms);
#if defined(CONFIG_RELAY)
/* Valid ordered intent only, not proof of physical power-down. The caller
 * supplies the same bounded ordinary membership/owner/publication tail in ms
 * on every call; the WoM advance is added by the event module. Call outside
 * receiver IRQ lock. */
uint32_t tracker_events_power_down_mask(uint32_t now_ms, uint32_t membership_tail_ms);
#endif
/* Invalidate identities while the pairing store's IRQ lock is held; scalar
 * generations only. Cleanup runs after IRQ unlock, before store mutex unlock. */
void tracker_events_pairing_invalidate(uint32_t tracker_mask);
void tracker_events_pairing_cleanup(void);
int tracker_events_control(const uint8_t *args, size_t len, uint32_t usb_generation,
                          uint8_t result[12]);
uint32_t tracker_events_usb_generation(void);
void tracker_events_usb_reset(void);
bool tracker_events_hid_valid(const uint8_t record[16], uint32_t generation, uint32_t now_ms);

#endif
