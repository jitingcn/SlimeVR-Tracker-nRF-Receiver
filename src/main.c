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
#include "globals.h"
#include "system/system.h"
#include "system/led.h"
#include "system/status.h"
#include "connection/esb.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
#include <zephyr/dfu/mcuboot.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static void boot_confirm_after_healthy_uptime(void)
{
#if defined(CONFIG_BOOTLOADER_MCUBOOT) || \
	(CONFIG_SOC_SERIES_NRF52 && CONFIG_BUILD_OUTPUT_UF2)
	int64_t remaining_ms = 60000 - k_uptime_get();
	if (remaining_ms > 0) {
		k_sleep(K_MSEC(remaining_ms));
	}
	sys_mark_boot_success();
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
	if (!boot_is_img_confirmed()) {
		int err = boot_write_img_confirmed();
		if (err) {
			LOG_ERR("Failed to confirm MCUboot image: %d", err);
		} else {
			LOG_INF("MCUboot test image confirmed");
		}
	}
#endif
#endif
}

int main(void)
{
	// Initialize LED system first
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);

	// Boot sequence with button detection
	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_BOOT);

	// Check if button is pressed during boot
	if (button_read())
	{
		LOG_INF("Button detected during boot");
		int64_t boot_start = k_uptime_get();
		bool pairing_reset_saved = false;

		while (button_read())
		{
			if (k_uptime_get() - boot_start > 1000)
				set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_HIGHEST);
			if (k_uptime_get() - boot_start > 5000)
			{
				LOG_INF("Boot-time pairing reset requested");
				pairing_reset_saved = esb_clear() == 0;
				break;
			}
			k_msleep(10);
		}

		if (pairing_reset_saved)
			set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_HIGHEST);
		else if (k_uptime_get() - boot_start > 50) // Short press during boot
			set_led(SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PRIORITY_HIGHEST);
	}

	// Clear boot LED priority
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_BOOT);
	boot_confirm_after_healthy_uptime();

	return 0;
}
