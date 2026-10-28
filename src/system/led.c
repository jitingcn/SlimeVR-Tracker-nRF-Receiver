#include "globals.h"
#include "util.h"

#include <errno.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "led.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static void led_thread(void);
/* The strip worker also executes synchronous SPI/driver and error-log paths. */
#if CONFIG_LED_STRIP
#define LED_THREAD_STACK_SIZE 1024
#else
#define LED_THREAD_STACK_SIZE 512
#endif
/* below ESB_THREAD_PRIORITY so LED cannot preempt radio */
K_THREAD_DEFINE(led_thread_id, LED_THREAD_STACK_SIZE, led_thread, NULL, NULL, NULL, LED_THREAD_PRIORITY, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if CONFIG_LED_STRIP
#define LED_STRIP_EXISTS 1
#include <zephyr/drivers/led_strip.h>
#define STRIP_NODE DT_ALIAS(led_strip)
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

static int led_strip_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
	struct led_rgb pixel = {
		.r = red,
		.g = green,
		.b = blue,
	};
	int err = led_strip_update_rgb(strip, &pixel, 1);

	if (err) {
		LOG_ERR("Failed to update LED strip: %d", err);
	}

	return err;
}
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#ifndef LED_EXISTS
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define LED0_EXISTS true
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#endif
#ifndef LED_EXISTS
#ifndef LED_STRIP_EXISTS
#warning "LED GPIO does not exist"
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
#define LED1_EXISTS true
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
#define LED2_EXISTS true
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
#define LED3_EXISTS true
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pwm_led0))
#define PWM_LED_EXISTS true
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
#else
#ifndef LED_STRIP_EXISTS
#warning "PWM LED node does not exist"
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led1))
#define PWM_LED1_EXISTS true
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led2))
#define PWM_LED2_EXISTS true
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
#endif

#if LED_EXISTS || LED_STRIP_EXISTS
static enum sys_led_pattern current_led_pattern = SYS_LED_PATTERN_OFF;
static int current_priority = SYS_LED_PATTERN_DEPTH;
static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH] = {[0 ... (SYS_LED_PATTERN_DEPTH - 1)] = SYS_LED_PATTERN_OFF};
static struct k_spinlock led_state_lock;
static K_SEM_DEFINE(led_changed, 0, 1);
static uint32_t led_generation = 1;
static uint32_t led_off_generation;
/* After boot-time pin initialization, only the LED thread accesses hardware. */
static int led_pattern_state;

/* A reboot requests OFF_FORCE immediately before resetting.  Do not wait
 * indefinitely if the LED device or its worker cannot make progress. */
#define LED_OFF_WAIT_MS 100

static int led_pin_init(void)
{
	LOG_DBG("led_pin_init");
#if LED_STRIP_EXISTS
	if (!device_is_ready(strip)) {
		LOG_ERR("LED strip device %s is not ready", strip->name);
		return -ENODEV;
	}

	led_strip_set_rgb(0, 0, 0);
#endif
	#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 0);
	#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	gpio_pin_set_dt(&led0, 0);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
	gpio_pin_set_dt(&led1, 0);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
	gpio_pin_set_dt(&led2, 0);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	gpio_pin_set_dt(&led3, 0);
#endif
	return 0;
}

SYS_INIT(led_pin_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static bool led_pin_reset(void)
{
	bool ok = true;
	LOG_DBG("led_pin_reset");
	#if LED_EXISTS
	ok &= gpio_pin_configure_dt(&led, GPIO_DISCONNECTED) == 0;
	#endif
#if LED0_EXISTS
	ok &= gpio_pin_configure_dt(&led0, GPIO_DISCONNECTED) == 0;
#endif
#if LED1_EXISTS
	ok &= gpio_pin_configure_dt(&led1, GPIO_DISCONNECTED) == 0;
#endif
#if LED2_EXISTS
	ok &= gpio_pin_configure_dt(&led2, GPIO_DISCONNECTED) == 0;
#endif
#if LED3_EXISTS
	ok &= gpio_pin_configure_dt(&led3, GPIO_DISCONNECTED) == 0;
#endif
	if (!ok) {
		LOG_ERR("Failed to disconnect LED pins");
	}
	return ok;
}


#if LED_STRIP_EXISTS
#define LED_RGB_COLOR
#else
#ifdef CONFIG_LED_RGB_COLOR
#define LED_RGB_COLOR
#define LED_RG_COLOR
#endif

#if PWM_LED_EXISTS && PWM_LED1_EXISTS && PWM_LED2_EXISTS
#define LED_TRI_COLOR
#else
#undef LED_RGB_COLOR
#undef LED_TRI_COLOR
#if PWM_LED_EXISTS && PWM_LED1_EXISTS
#define LED_DUAL_COLOR
#else
#undef LED_RG_COLOR
#undef LED_DUAL_COLOR
#endif
#endif
#endif

#ifdef LED_RGB_COLOR
static const int led_pwm_period[5][3] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G, CONFIG_LED_DEFAULT_COLOR_B}, // Default
	{0, 10000, 0}, // Success
	{10000, 0, 0}, // Error
	{8000, 2000, 0}, // Charging
	{0, 0, 10000}, // Pairing
};
#elif defined(LED_TRI_COLOR)
static const int led_pwm_period[5][3] = {
	{0, 0, 10000}, // Default
	{0, 10000, 0}, // Success
	{10000, 0, 0}, // Error
	{6000, 4000, 0}, // Charging
	{0, 0, 10000}, // Pairing
};
#elif defined(LED_RG_COLOR)
static const int led_pwm_period[5][2] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G}, // Default
	{0, 10000}, // Success
	{10000, 0}, // Error
	{8000, 2000}, // Charging
	{4000, 6000}, // Pairing
};
#elif defined(LED_DUAL_COLOR)
static const int led_pwm_period[5][2] = {
	{0, 10000}, // Default
	{0, 10000}, // Success
	{10000, 0}, // Error
	{6000, 4000}, // Charging
	{0, 10000}, // Pairing
};
#else
static const int led_pwm_period[5][1] = {
	{10000}, // Default
	{10000}, // Success
	{10000}, // Error
	{10000}, // Charging
	{10000}, // Pairing
};
#endif

// Using brightness and value if PWM is supported, otherwise value is coerced to on/off
// TODO: use computed constants for high/low brightness and color values
static void led_pin_set(enum sys_led_color color, int brightness_pptt, int value_pptt)
{
	LOG_DBG("led_pin_set: color %d, brightness %d, value %d", color, brightness_pptt, value_pptt);
	if (brightness_pptt < 0)
		brightness_pptt = 0;
	else if (brightness_pptt > 10000)
		brightness_pptt = 10000;
	if (value_pptt < 0)
		value_pptt = 0;
	else if (value_pptt > 10000)
		value_pptt = 10000;
#if LED_STRIP_EXISTS
	value_pptt = value_pptt * brightness_pptt / 10000;
	led_strip_set_rgb(
		255 * (led_pwm_period[color][0] * value_pptt / 10000) / 10000,
		255 * (led_pwm_period[color][1] * value_pptt / 10000) / 10000,
		255 * (led_pwm_period[color][2] * value_pptt / 10000) / 10000
	);
#elif PWM_LED_EXISTS
	value_pptt = value_pptt * brightness_pptt / 10000;
	// only supporting color if PWM is supported
	pwm_set_pulse_dt(&pwm_led, pwm_led.period / 10000 * (led_pwm_period[color][0] * value_pptt / 10000));
#if PWM_LED1_EXISTS
	pwm_set_pulse_dt(&pwm_led1, pwm_led1.period / 10000 * (led_pwm_period[color][1] * value_pptt / 10000));
#if PWM_LED2_EXISTS
	pwm_set_pulse_dt(&pwm_led2, pwm_led2.period / 10000 * (led_pwm_period[color][2] * value_pptt / 10000));
#endif
#endif
#else
	gpio_pin_set_dt(&led, value_pptt > 5000);
#endif
}

/* Caller holds led_state_lock. OFF yields to lower priorities, while
 * OFF_FORCE is an active selection which masks them. */
static bool led_select_locked(enum sys_led_pattern pattern, int priority)
{
	led_patterns[priority] = pattern;
	for (priority = 0; priority < SYS_LED_PATTERN_DEPTH; priority++) {
		if (led_patterns[priority] != SYS_LED_PATTERN_OFF) {
			break;
		}
	}
	pattern = priority < SYS_LED_PATTERN_DEPTH ? led_patterns[priority] : SYS_LED_PATTERN_OFF;
	if (pattern == current_led_pattern && priority == current_priority) {
		return false;
	}
	current_led_pattern = pattern;
	current_priority = priority;
	led_generation++;
	return true;
}

/* A completed one-shot may only clear the selection it actually rendered. */
static void led_finish_oneshot(uint32_t generation, int priority, enum sys_led_pattern finish)
{
	k_spinlock_key_t key = k_spin_lock(&led_state_lock);
	bool changed = generation == led_generation && priority == current_priority
		&& led_select_locked(finish, priority);
	k_spin_unlock(&led_state_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
}
#endif

void set_led(enum sys_led_pattern led_pattern, int priority)
{
	LOG_DBG("set_led: pattern %d, priority %d", led_pattern, priority);
#if LED_EXISTS || LED_STRIP_EXISTS
	k_spinlock_key_t key = k_spin_lock(&led_state_lock);
	bool changed = led_select_locked(led_pattern, priority);
	uint32_t generation = led_generation;
	bool wait_for_off = led_pattern == SYS_LED_PATTERN_OFF_FORCE
		&& current_led_pattern == SYS_LED_PATTERN_OFF_FORCE && current_priority == priority;
	k_spin_unlock(&led_state_lock, key);

	if (changed) {
		k_sem_give(&led_changed);
	}

	/* Most requests are asynchronous and are safe even from an ISR. A
	 * thread requesting forced-off waits only for the worker's black frame:
	 * sys_request_system_reboot() resets immediately after this call. Never
	 * wait from an ISR or from the hardware-owning thread itself. */
	if (wait_for_off && !k_is_in_isr() && k_current_get() != led_thread_id) {
		int64_t deadline = k_uptime_get() + LED_OFF_WAIT_MS;
		while (true) {
			key = k_spin_lock(&led_state_lock);
			bool applied = led_off_generation == generation;
			bool superseded = led_generation != generation;
			k_spin_unlock(&led_state_lock, key);
			if (applied) {
				break;
			}
			if (superseded || k_uptime_get() >= deadline) {
				LOG_ERR("LED forced-off was not applied before reboot");
				break;
			}
			k_msleep(1);
		}
	}
#else
	ARG_UNUSED(led_pattern);
	ARG_UNUSED(priority);
#endif
}

static void led_thread(void)
{
#if !LED_EXISTS && !LED_STRIP_EXISTS
	LOG_WRN("LED GPIO does not exist");
	return;
#else
	uint32_t active_generation = 0;
	while (1)
	{
		k_spinlock_key_t key = k_spin_lock(&led_state_lock);
		enum sys_led_pattern pattern = current_led_pattern;
		int priority = current_priority;
		uint32_t generation = led_generation;
		k_spin_unlock(&led_state_lock, key);

		if (active_generation != generation) {
			active_generation = generation;
			led_pattern_state = 0;
			if (pattern <= SYS_LED_PATTERN_OFF) {
				bool off_applied = true;
#if LED_STRIP_EXISTS
				off_applied = led_strip_set_rgb(0, 0, 0) == 0;
#endif
				off_applied &= led_pin_reset();
				/* Do not acknowledge a failed or obsolete black frame. */
				key = k_spin_lock(&led_state_lock);
				if (off_applied && generation == led_generation) {
					led_off_generation = generation;
				}
				k_spin_unlock(&led_state_lock, key);
			} else {
				led_pin_init();
			}
		}

		int wait_ms = -1;
		switch (pattern)
		{
		case SYS_LED_PATTERN_ON:
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 10000);
			break;
		case SYS_LED_PATTERN_SHORT:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_PAIRING, 10000, led_pattern_state * 10000);
			wait_ms = led_pattern_state == 1 ? 100 : 900;
			break;
		case SYS_LED_PATTERN_LONG:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, led_pattern_state * 10000);
			wait_ms = 500;
			break;
		case SYS_LED_PATTERN_FLASH:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, led_pattern_state * 10000);
			wait_ms = 200;
			break;

		case SYS_LED_PATTERN_ONESHOT_POWERON:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 7)
				led_finish_oneshot(generation, priority, SYS_LED_PATTERN_OFF);
			else
				wait_ms = 200;
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0)
				led_pin_set(SYS_LED_COLOR_DEFAULT, (202 - led_pattern_state) * 50, (led_pattern_state != 202 ? 10000 : 0));
			else
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
			if (led_pattern_state == 202)
				led_finish_oneshot(generation, priority, SYS_LED_PATTERN_OFF_FORCE);
			else if (led_pattern_state == 1)
				wait_ms = 250;
			else
				wait_ms = 5;
			break;
		case SYS_LED_PATTERN_ONESHOT_PROGRESS:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 5)
				led_finish_oneshot(generation, priority, SYS_LED_PATTERN_OFF);
			else
				wait_ms = 200;
			break;
		case SYS_LED_PATTERN_ONESHOT_COMPLETE:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 9)
				led_finish_oneshot(generation, priority, SYS_LED_PATTERN_OFF);
			else
				wait_ms = 200;
			break;

		case SYS_LED_PATTERN_ON_PERSIST:
			led_pin_set(SYS_LED_COLOR_SUCCESS, 2000, 10000);
			break;
		case SYS_LED_PATTERN_LONG_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_CHARGING, 2000, led_pattern_state * 10000);
			wait_ms = 500;
			break;
		case SYS_LED_PATTERN_PULSE_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 1000;
			int led_value = led_pattern_state > 500 ? 1000 - led_pattern_state : led_pattern_state;
			if (led_value < 200)
				led_value = (led_value) * 30;
			else if (led_value < 300)
				led_value = (led_value - 200) * 20 + 6000;
			else if (led_value < 400)
				led_value = (led_value - 300) * 15 + 8000;
			else
				led_value = (led_value - 400) * 5 + 9500;
			led_pin_set(SYS_LED_COLOR_CHARGING, 10000, led_value);
			wait_ms = 5;
			break;
		case SYS_LED_PATTERN_ACTIVE_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, !led_pattern_state * 10000);
			wait_ms = led_pattern_state ? 9700 : 300;
			break;

		case SYS_LED_PATTERN_ERROR_A:
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 4 && led_pattern_state % 2) * 10000);
			wait_ms = 500;
			break;
		case SYS_LED_PATTERN_ERROR_B:
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 6 && led_pattern_state % 2) * 10000);
			wait_ms = 500;
			break;
		case SYS_LED_PATTERN_ERROR_C:
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 8 && led_pattern_state % 2) * 10000);
			wait_ms = 500;
			break;
		case SYS_LED_PATTERN_ERROR_D:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, led_pattern_state * 10000);
			wait_ms = 500;
			break;

		default:
			break;
		}

		/* Discard a notification already reflected in the snapshot, then
		 * recheck under the lock so a concurrent change cannot be lost. */
		(void)k_sem_take(&led_changed, K_NO_WAIT);
		key = k_spin_lock(&led_state_lock);
		bool unchanged = generation == led_generation;
		k_spin_unlock(&led_state_lock, key);
		if (unchanged) {
			(void)k_sem_take(&led_changed, wait_ms < 0 ? K_FOREVER : K_MSEC(wait_ms));
		}
	}
#endif
}
