/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

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
#include "usb.h"
#include "console.h"
#include "data_collect.h"

#include <hal/nrf_power.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log_ctrl.h>
#include "system/system.h"
#include "system/status.h"
#include "thread_priority.h"

#define DFU_EXISTS (CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER || CONFIG_BOOTLOADER_MCUBOOT)
#define ADAFRUIT_DFU_MAGIC_SERIAL_ONLY_RESET 0x4E

static void usb_ctrl_thread(void);
static int64_t usb_start_time_ms;
static K_SEM_DEFINE(usb_ctrl_start_sem, 0, 1);
static K_MUTEX_DEFINE(usb_serial_transition_lock);

LOG_MODULE_REGISTER(usb, LOG_LEVEL_INF);

#define SLIMEVR_USB_STRING_MANUFACTURER_IDX 1U
#define SLIMEVR_USB_STRING_PRODUCT_IDX 2U
#define SLIMEVR_USB_STRING_SERIAL_NUMBER_IDX COND_CODE_1(CONFIG_HWINFO, (3U), (0U))

#define SLIMEVR_USBD_DEVICE_DEFINE(device_name, udc_dev, vid, pid)	\
	static struct usb_device_descriptor				\
	fs_desc_##device_name = {					\
		.bLength = sizeof(struct usb_device_descriptor),	\
		.bDescriptorType = USB_DESC_DEVICE,			\
		.bcdUSB = sys_cpu_to_le16(USB_SRN_2_0),			\
		.bDeviceClass = USB_BCC_MISCELLANEOUS,			\
		.bDeviceSubClass = 2,					\
		.bDeviceProtocol = 1,					\
		.bMaxPacketSize0 = USB_CONTROL_EP_MPS,			\
		.idVendor = vid,					\
		.idProduct = pid,					\
		.bcdDevice = sys_cpu_to_le16(USB_BCD_DRN),		\
		.iManufacturer = SLIMEVR_USB_STRING_MANUFACTURER_IDX,	\
		.iProduct = SLIMEVR_USB_STRING_PRODUCT_IDX,		\
		.iSerialNumber = SLIMEVR_USB_STRING_SERIAL_NUMBER_IDX,	\
		.bNumConfigurations = 0,				\
	};								\
	IF_ENABLED(USBD_SUPPORTS_HIGH_SPEED, (				\
	static struct usb_device_descriptor				\
	hs_desc_##device_name = {					\
		.bLength = sizeof(struct usb_device_descriptor),	\
		.bDescriptorType = USB_DESC_DEVICE,			\
		.bcdUSB = sys_cpu_to_le16(USB_SRN_2_0),			\
		.bDeviceClass = USB_BCC_MISCELLANEOUS,			\
		.bDeviceSubClass = 2,					\
		.bDeviceProtocol = 1,					\
		.bMaxPacketSize0 = 64,					\
		.idVendor = vid,					\
		.idProduct = pid,					\
		.bcdDevice = sys_cpu_to_le16(USB_BCD_DRN),		\
		.iManufacturer = SLIMEVR_USB_STRING_MANUFACTURER_IDX,	\
		.iProduct = SLIMEVR_USB_STRING_PRODUCT_IDX,		\
		.iSerialNumber = SLIMEVR_USB_STRING_SERIAL_NUMBER_IDX,	\
		.bNumConfigurations = 0,				\
	};								\
	))								\
	static STRUCT_SECTION_ITERABLE(usbd_context, device_name) = {	\
		.name = STRINGIFY(device_name),				\
		.dev = udc_dev,						\
		.fs_desc = &fs_desc_##device_name,			\
		IF_ENABLED(USBD_SUPPORTS_HIGH_SPEED, (			\
		.hs_desc = &hs_desc_##device_name,			\
		))							\
	}

SLIMEVR_USBD_DEVICE_DEFINE(slimevr_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
			   CONFIG_SLIMEVR_USB_DEVICE_VID, CONFIG_SLIMEVR_USB_DEVICE_PID);

USBD_DESC_LANG_DEFINE(slimevr_lang);
USBD_DESC_MANUFACTURER_DEFINE(slimevr_mfr, CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(slimevr_product, CONFIG_SLIMEVR_USB_DEVICE_PRODUCT);
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(slimevr_sn)));

USBD_DESC_CONFIG_DEFINE(slimevr_fs_cfg_desc, "FS Configuration");
USBD_CONFIGURATION_DEFINE(slimevr_fs_config, 0, CONFIG_SLIMEVR_USB_DEVICE_MAX_POWER,
			  &slimevr_fs_cfg_desc);

static bool usb_enabled;
static atomic_t usb_configured;
static atomic_t usb_config_epoch;
static receiver_usb_state_cb_t receiver_usb_state_callback;

static void receiver_usb_set_configured(bool configured)
{
	bool was_configured = atomic_get(&usb_configured) != 0;

	if (was_configured != configured) {
		atomic_inc(&usb_config_epoch);
	}
	atomic_set(&usb_configured, configured ? 1 : 0);

	if (receiver_usb_state_callback != NULL) {
		receiver_usb_state_callback(configured);
	}
}

bool receiver_usb_is_enabled(void)
{
	return usb_enabled;
}

bool receiver_usb_is_configured(void)
{
	return atomic_get(&usb_configured) != 0;
}

void receiver_usb_set_state_callback(receiver_usb_state_cb_t callback)
{
	receiver_usb_state_callback = callback;
}

static int usbd_add_string_descriptors(void)
{
	int ret;

	ret = usbd_add_descriptor(&slimevr_usbd, &slimevr_lang);
	if (ret != 0) {
		return ret;
	}

	ret = usbd_add_descriptor(&slimevr_usbd, &slimevr_mfr);
	if (ret != 0) {
		return ret;
	}

	ret = usbd_add_descriptor(&slimevr_usbd, &slimevr_product);
	if (ret != 0) {
		return ret;
	}

	IF_ENABLED(CONFIG_HWINFO, (ret = usbd_add_descriptor(&slimevr_usbd, &slimevr_sn);))

	return ret;
}

static int usbd_setup(usbd_msg_cb_t msg_cb)
{
	int ret;

	ret = usbd_add_string_descriptors();
	if (ret != 0) {
		LOG_ERR("Failed to add USB string descriptors: %d", ret);
		return ret;
	}

	ret = usbd_add_configuration(&slimevr_usbd, USBD_SPEED_FS, &slimevr_fs_config);
	if (ret != 0) {
		LOG_ERR("Failed to add USB FS configuration: %d", ret);
		return ret;
	}

	ret = usbd_register_all_classes(&slimevr_usbd, USBD_SPEED_FS, 1, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to register USB classes: %d", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS)) {
		ret = usbd_device_set_code_triple(&slimevr_usbd, USBD_SPEED_FS,
						  USB_BCC_MISCELLANEOUS, 0x02, 0x01);
		if (ret != 0) {
			LOG_ERR("Failed to set USB code triple: %d", ret);
			return ret;
		}
	}

	ret = usbd_msg_register_cb(&slimevr_usbd, msg_cb);
	if (ret != 0) {
		LOG_ERR("Failed to register USB message callback: %d", ret);
		return ret;
	}

	ret = usbd_init(&slimevr_usbd);
	if (ret != 0) {
		LOG_ERR("Failed to initialize USB device: %d", ret);
	}

	return ret;
}

static bool usb_vbus_present(void)
{
#ifdef POWER_USBREGSTATUS_VBUSDETECT_Msk
	return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
#else
	return false;
#endif
}

static int usb_enable_device(struct usbd_context *ctx)
{
	if (usb_enabled) {
		return 0;
	}

	int ret = usbd_enable(ctx);

	if (ret == -EALREADY) {
		usb_enabled = true;
		return 0;
	}

	if (ret == 0) {
		receiver_usb_set_configured(false);
		usb_enabled = true;
	}

	return ret;
}

static void usb_close_serial(bool invalidate)
{
	k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
	if (get_status(SYS_STATUS_SERIAL_ACTIVE)) {
		set_status(SYS_STATUS_SERIAL_ACTIVE, false);
	}
	if (invalidate) {
		console_serial_stop();
	} else {
		console_serial_close();
	}
	log_backend_disable(log_backend_get_by_name("log_backend_uart"));
	k_mutex_unlock(&usb_serial_transition_lock);
}

static void status_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	int ret;
	int disable_ret;
	switch (msg->type) {
	case USBD_MSG_RESET:
		receiver_usb_set_configured(false);
		usb_close_serial(true);
		break;
	case USBD_MSG_CONFIGURATION:
		receiver_usb_set_configured(msg->status != 0);
		if (msg->status == 0) {
			usb_close_serial(true);
		}
		break;
	case USBD_MSG_VBUS_READY:
		ret = usb_enable_device(ctx);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB device: %d", ret);
		}
		break;
	case USBD_MSG_VBUS_REMOVED:
		receiver_usb_set_configured(false);
		usb_close_serial(true);
		usb_enabled = false;
		disable_ret = usbd_disable(ctx);
		if (disable_ret != 0) {
			LOG_ERR("Failed to disable USB device: %d", disable_ret);
		}
		break;
	default:
		LOG_DBG("USBD message %s unhandled", usbd_msg_type_string(msg->type));
		break;
	}
}

static void usb_init_thread(void)
{
	int ret;

#if defined(CONFIG_DATA_COLLECT) && !defined(CONFIG_DATA_COLLECT_HID)
	ret = data_collect_init();
	if (ret != 0) {
		return;
	}
#endif

	ret = usbd_setup(status_cb);
	if (ret != 0) {
		return;
	}

	usb_start_time_ms = k_uptime_get();
	if (!usbd_can_detect_vbus(&slimevr_usbd) || usb_vbus_present()) {
		ret = usb_enable_device(&slimevr_usbd);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB device: %d", ret);
			return;
		}
	}

	k_sem_give(&usb_ctrl_start_sem);
}

static void usb_ctrl_thread(void)
{
	const struct log_backend *const backend = log_backend_get_by_name("log_backend_uart");
	const struct device *const dev_console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr;
	uint32_t last_dtr = 0;
	uint32_t seen_config_epoch = 0;
	bool serial_active;
#if DFU_EXISTS
	bool button_checked_for_config = false;
	bool button_raw_recheck_pending = false;
#endif

	k_sem_take(&usb_ctrl_start_sem, K_FOREVER);

	// The watcher is permanent; USB lifecycle events only gate its polling.
	while (1) {
		uint32_t config_epoch = (uint32_t)atomic_get(&usb_config_epoch);

		if (!receiver_usb_is_configured()) {
			last_dtr = 0;
			seen_config_epoch = config_epoch;
#if DFU_EXISTS
			button_checked_for_config = false;
			button_raw_recheck_pending = false;
#endif
			k_msleep(10);
			continue;
		}

		if (config_epoch != seen_config_epoch) {
			seen_config_epoch = config_epoch;
			last_dtr = 0;
#if DFU_EXISTS
			button_checked_for_config = false;
			button_raw_recheck_pending = false;
#endif
		}

#if DFU_EXISTS
		if (!button_checked_for_config) {
			int64_t since_start_ms = k_uptime_get() - usb_start_time_ms;
			bool enter_dfu = since_start_ms < 100
				? button_read_filtered()
				: button_read();
			button_checked_for_config = true;
			button_raw_recheck_pending = since_start_ms < 100;
			if (enter_dfu) {
				sys_enter_dfu(false);
			}
		} else if (button_raw_recheck_pending &&
			   k_uptime_get() - usb_start_time_ms >= 100) {
			button_raw_recheck_pending = false;
			if (button_read()) {
				sys_enter_dfu(false);
			}
		}
#endif

		dtr = 0;
		if (uart_line_ctrl_get(dev_console, UART_LINE_CTRL_DTR, &dtr) != 0) {
			dtr = 0;
		}
		serial_active = get_status(SYS_STATUS_SERIAL_ACTIVE);
		if (dtr == last_dtr && ((dtr != 0U) == serial_active)) {
			k_msleep(10);
			continue;
		}
		last_dtr = dtr;

		if (dtr) {
			k_mutex_lock(&usb_serial_transition_lock, K_FOREVER);
			if (receiver_usb_is_configured()) {
				set_status(SYS_STATUS_SERIAL_ACTIVE, true);
				console_serial_start();
				log_backend_enable(backend, backend->cb->ctx, CONFIG_LOG_MAX_LEVEL);
			}
			k_mutex_unlock(&usb_serial_transition_lock);
		} else {
#if CONFIG_BUILD_OUTPUT_UF2
			// Adafruit UF2 supports the 1200 baud serial-only DFU touch.
			uint32_t baudrate = 0;
			bool request_dfu = uart_line_ctrl_get(dev_console, UART_LINE_CTRL_BAUD_RATE,
							      &baudrate) == 0 && baudrate == 1200;
			if (request_dfu) {
				NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_SERIAL_ONLY_RESET;
				usb_close_serial(true);
				k_msleep(100);
				sys_request_system_reboot();
				continue;
			}
#endif
			usb_close_serial(false);
		}
	}
}

/* below ESB_THREAD_PRIORITY; the USB control watcher stays alive across
 * configuration changes, while one-shot USB init must not outrank radio. */
K_THREAD_DEFINE(usb_ctrl_thread_id, 512, usb_ctrl_thread, NULL, NULL, NULL,
		USB_INIT_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(usb_init_thread_id, 512, usb_init_thread, NULL, NULL, NULL,
		USB_INIT_THREAD_PRIORITY, 0, 500);
