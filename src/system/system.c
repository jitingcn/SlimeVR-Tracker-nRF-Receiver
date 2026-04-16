#include "globals.h"
#include "connection/esb.h"
#include "system/led.h"
#include "system/status.h"
#include "retained.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "system.h"

static struct nvs_fs fs;

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

// Button support
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)
#define BUTTON_EXISTS true
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static int64_t press_time = 0;
static int64_t last_press_duration = 0;
static void button_thread(void);
K_THREAD_DEFINE(button_thread_id, 1024, button_thread, NULL, NULL, NULL, 6, 0, 0);
#else
#define BUTTON_EXISTS false
#pragma message "Button GPIO does not exist"
#endif

// DFU support check
#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

static bool nvs_init = false;

static int sys_nvs_init(void) {
	if (nvs_init) {
		return 0;
	}
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET;  // starting at NVS_PARTITION_OFFSET
	if (flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info)) {
		LOG_ERR("Failed to get page info");
		return 1;
	}
	fs.sector_size = info.size;  // sector_size equal to the pagesize
	fs.sector_count = 4U;  // 4 sectors
	int err = nvs_mount(&fs);
	if (err == -EDEADLK) {
		LOG_WRN("All sectors closed, erasing all sectors...");
		err = flash_flatten(
			fs.flash_device,
			fs.offset,
			fs.sector_size * fs.sector_count
		);
		if (!err) {
			err = nvs_mount(&fs);
		}
	}
	if (err) {
		LOG_ERR("Failed to mount NVS");
		return 1;
	}
	nvs_init = true;
	return 0;
}

SYS_INIT(sys_nvs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// TODO: switch back to retained?
uint8_t reboot_counter_read(void) {
	uint8_t reboot_counter;
	nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	return reboot_counter;
}

#if BUTTON_EXISTS
static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bool pressed = button_read();
	int64_t current_time = k_uptime_get();
	if (press_time && !pressed && current_time - press_time > 50) // debounce
		last_press_duration = current_time - press_time;
	else if (press_time && pressed) // unusual press event on button already pressed
		return;
	press_time = pressed ? current_time : 0;
}

static struct gpio_callback button_cb_data;

static int sys_button_init(void)
{
	gpio_pin_configure_dt(&button0, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button_cb_data);
	return 0;
}

SYS_INIT(sys_button_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

bool button_read(void)
{
#if BUTTON_EXISTS
	return gpio_pin_get_dt(&button0);
#else
	return false;
#endif
}

void sys_request_system_off(void)
{
	LOG_INF("System shutdown requested");
	reboot_counter_write(0);
	set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
	k_msleep(2000);
	sys_reboot(SYS_REBOOT_COLD);
}

void sys_request_system_reboot(void)
{
	LOG_INF("System reboot requested");
	sys_reboot(SYS_REBOOT_COLD);
}

#if BUTTON_EXISTS
static void button_thread(void)
{
	int num_presses = 0;
	int64_t last_press = 0;
	int64_t press_start_time = 0;
	bool long_press_5s_handled = false;
	bool long_press_10s_handled = false;
	int64_t last_blink_time = 0;
	bool led_state = false;
	bool long_press_5s_triggered = false; // Track if 5s function has been triggered

	while (1)
	{
		// Handle button press start - immediate response
		if (press_time && !press_start_time)
		{
			press_start_time = press_time;
			long_press_5s_handled = false;
			long_press_10s_handled = false;
			long_press_5s_triggered = false;
			set_status(SYS_STATUS_BUTTON_PRESSED, true);
			// Immediate LED response
			set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
			last_blink_time = k_uptime_get();
			led_state = true;
			LOG_INF("Button press started");
		}

		// Handle button press duration - blink every second to confirm time
		if (press_time && button_read())
		{
			int64_t current_time = k_uptime_get();
			int64_t hold_duration = current_time - press_start_time;

			// Blink every second to confirm timing
			if (current_time - last_blink_time >= 1000)
			{
				led_state = !led_state;
				if (led_state)
					set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
				else
					set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
				last_blink_time = current_time;
				LOG_INF("Button held for %lld seconds", hold_duration / 1000);
			}

			// Handle long press actions with proper sequencing
			if (hold_duration >= 10000 && !long_press_10s_handled) // 10 seconds - DFU mode
			{
				LOG_INF("DFU mode requested (10s)");
				set_led(SYS_LED_PATTERN_ERROR_D, SYS_LED_PRIORITY_HIGHEST);
#if DFU_EXISTS
#if CONFIG_BUILD_OUTPUT_UF2 // Adafruit bootloader
				NRF_POWER->GPREGRET = 0x57;
#elif CONFIG_BOARD_HAS_NRF5_BOOTLOADER // NRF5 bootloader
				// Use GPIO method to enter DFU
				const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
				if (device_is_ready(gpio_dev))
				{
					gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
				}
#else
				// Use double reset memory method as fallback
				*dbl_reset_mem = DFU_DBL_RESET_APP;
				ram_range_retain(dbl_reset_mem, sizeof(*dbl_reset_mem), true);
#endif
				// Wait a bit to ensure data is written
				k_msleep(100);
#endif
				// System reboot to enter DFU
				sys_request_system_reboot();
				long_press_10s_handled = true;
				return; // Exit thread as system will reboot
			}
			// Note: 5s function is no longer triggered during hold, only checked on release
		}

		// Handle button release
		if (last_press_duration > 50 && press_start_time) // debounce
		{
			int64_t press_duration = last_press_duration;
			last_press_duration = 0;

			// Check for long press (execute 5s function only on release)
			if (press_duration >= 5000 && press_duration < 10000 && !long_press_5s_triggered)
			{
				LOG_INF("Clear all pairings requested (5s) - triggered on release");
				esb_clear();
				set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_HIGHEST);
				long_press_5s_triggered = true;
				// Reset counters
				num_presses = 0;
				last_press = 0;
			}
			else if (press_duration < 5000) // Only short presses count for press sequences
			{
				num_presses++;
				LOG_INF("Button pressed %d times (duration: %lldms)", num_presses, press_duration);
				last_press = k_uptime_get();
			}
			else
			{
				LOG_INF("Long press completed (duration: %lldms)", press_duration);
				// Reset counters
				num_presses = 0;
				last_press = 0;
			}

			press_start_time = 0;

			// Clear button pressed status and LED
			set_status(SYS_STATUS_BUTTON_PRESSED, false);
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			long_press_5s_handled = false;
			long_press_10s_handled = false;
		}

		// Handle multiple press patterns (only for short presses)
		if (last_press && k_uptime_get() - last_press > 1000 && num_presses > 0)
		{
			LOG_INF("Button sequence completed: %d presses", num_presses);

			switch (num_presses)
			{
			case 1: // Shutdown all active trackers
				LOG_INF("Shutdown all trackers requested");
				esb_request_all_shutdown();
				set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
				break;

			case 2: // Exit pairing mode
				LOG_INF("Exit pairing mode requested");
				esb_finish_pair();
				set_status(SYS_STATUS_PAIRING_MODE, false);
				set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
				break;

			case 3: // Enter pairing mode (non-blocking)
				LOG_INF("Enter pairing mode requested");
				set_status(SYS_STATUS_PAIRING_MODE, true);
				// Use non-blocking pairing mode
				esb_start_pairing();
				break;

			default:
				LOG_WRN("Unknown button sequence: %d presses", num_presses);
				break;
			}

			num_presses = 0;
			last_press = 0;
		}

		k_msleep(50); // Reduce check interval to improve responsiveness
	}
}
#endif

void reboot_counter_write(uint8_t reboot_counter) {
	nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
}

// retained not implemented
void sys_write(uint16_t id, void* retained_ptr, const void* data, size_t len) {
	sys_nvs_init();
	int err = nvs_write(&fs, id, data, len);
	if (err < 0)
	{
		LOG_ERR("Failed to write to NVS, error: %d", err);
		return;
	}
}

// reading from nvs
void sys_read(uint16_t id, void* data, size_t len) {
	sys_nvs_init();
	int err = nvs_read(&fs, id, data, len);
	if (err < 0)
	{
		if (err == -ENOENT) // suppress ENOENT
		{
			LOG_DBG("No entry exists for ID %d, read data set to zero", id);
		}
		else
		{
			LOG_ERR("Failed to read from NVS, error: %d", err);
			LOG_WRN("Read data set to zero");
		}
		memset(data, 0, len);
		return;
	}
}
