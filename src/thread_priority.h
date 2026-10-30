/* Zephyr: lower number = higher priority.
 * Negative = cooperative; non-negative = preemptible (yields on k_sleep).
 *
 * Fixed outside this file:
 *   RADIO IRQ=1, ESB EVENT IRQ=2, system WQ=-1 (HID submit), main=0
 *
 * App preempt ladder (keep gaps small; NUM_PREEMPT_PRIORITIES=15):
 *   7  ESB housekeeping — above UI / flash / logging
 *   8  UI + light services
 *   9  Background (flash, NVS, long scans, stats logging)
 */
#define ESB_THREAD_PRIORITY 7

#define ESB_STATS_THREAD_PRIORITY 8
#define CONSOLE_THREAD_PRIORITY 8
#define USB_INIT_THREAD_PRIORITY 8
#define LED_THREAD_PRIORITY 8
#define STATUS_THREAD_PRIORITY 8
#define BUTTON_THREAD_PRIORITY 8
#define AUTO_CHANNEL_THREAD_PRIORITY 8

#define RCV_OTA_WRITER_PRIORITY 9
#define HID_DROPPED_REPORTS_LOGGING_PRIORITY 9
#define RSSI_WQ_PRIORITY 9
