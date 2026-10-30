#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <setjmp.h>

#define ARG_UNUSED(x) (void)(x)
#define CODE_UNREACHABLE __builtin_unreachable()
#define SYS_REBOOT_COLD 0
#define K_MSEC(x) (x)
#ifndef TEST_GENERATED_CONFIG
#define CONFIG_BOARD_HAS_NRF5_BOOTLOADER 0
#ifdef TEST_UF2
#define CONFIG_SOC_SERIES_NRF52 1
#define CONFIG_BUILD_OUTPUT_UF2 1
#elif defined(TEST_LEGACY)
#define CONFIG_SOC_SERIES_NRF52 1
#define CONFIG_BUILD_OUTPUT_UF2 0
#else
#define CONFIG_SOC_SERIES_NRF52 0
#define CONFIG_BUILD_OUTPUT_UF2 0
#define CONFIG_BOOTLOADER_MCUBOOT 1
#endif
#endif

static struct { volatile uint32_t GPREGRET, GPREGRET2, RESETREAS; } power_regs;
#define NRF_POWER (&power_regs)
static struct { volatile uint32_t CC[4]; } timer_regs;
#define NRF_TIMER2 (&timer_regs)
static bool recovery_capable;
static uint32_t capability_token;
#ifdef TEST_MCUBOOT
static struct { volatile uint32_t RESETREAS; } reset_regs;
#define NRF_RESET (&reset_regs)
#define RESET_REGISTER reset_regs.RESETREAS
#else
#define RESET_REGISTER power_regs.RESETREAS
#endif
#define POWER_RESETREAS_DOG_Msk (1u << 1)
#define POWER_RESETREAS_VBUS_Msk (1u << 20)
#define RESET_RESETREAS_VBUS_Msk POWER_RESETREAS_VBUS_Msk
static unsigned locks, barriers, panics, logs, reboots, confirms;
static uint32_t inject_marker;
static unsigned int irq_lock(void) {
    assert(locks == 0);
    if (inject_marker) { power_regs.GPREGRET = inject_marker; inject_marker = 0; }
    return locks++;
}
static void irq_unlock(unsigned int key) { assert(locks == 1 && key == 0); locks = 0; }
static void barrier(void) {
    assert(power_regs.GPREGRET == (recovery_capable ? 0xE3u : 0x57u));
    barriers++;
}
#define __DSB() barrier()
#define LOG_PANIC() (++panics)
#define LOG_ERR(...) (++logs)
#define LOG_INF(...) (++logs)
struct arch_esf { int unused; };
static jmp_buf reboot_exit;
static void sys_arch_reboot(int type) {
    assert(type == SYS_REBOOT_COLD);
#ifdef TEST_UF2
    assert(barriers == 1); /* Marker publication precedes the direct arch reset. */
#endif
    reboots++;
    longjmp(reboot_exit, 1);
}
static int64_t now;
static uint32_t sleep_marker;
static unsigned sleeps, led_calls;
static bool confirmed;
static int confirm_error;
static int64_t k_uptime_get(void) { return now; }
static void k_sleep(int64_t ms) {
    assert(ms > 0 && now < 60000 && ms == 60000 - now);
    assert(power_regs.GPREGRET == sleep_marker && led_calls > 0);
    sleeps++;
    now += ms;
}
static void k_msleep(unsigned ms) { now += ms; }
static bool boot_is_img_confirmed(void) { assert(now >= 60000); return confirmed; }
static int boot_write_img_confirmed(void) { confirms++; return confirm_error; }
enum {
    SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM,
    SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_BOOT, SYS_LED_PATTERN_LONG,
    SYS_LED_PRIORITY_HIGHEST, SYS_LED_PATTERN_ONESHOT_COMPLETE,
    SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PATTERN_OFF,
};
static void set_led(int pattern, int priority) {
    (void)pattern; (void)priority; led_calls++;
}
static bool button_read(void) { return false; }
static int esb_clear(void) { return 0; }

struct gpio_callback { int unused; };
static struct gpio_callback button_cb_data;
static const struct { void *port; unsigned pin; } button0 = {NULL, 3};
static bool button_held_from_init;
static unsigned pin_reads;
#define GPIO_INPUT 0
#define GPIO_INT_EDGE_BOTH 1
#define BIT(x) (1u << (x))
#define gpio_pin_configure_dt(pin, mode) ((void)(pin), (void)(mode))
#define gpio_pin_interrupt_configure_dt(pin, mode) ((void)(pin), (void)(mode))
#define gpio_init_callback(cb, fn, mask) ((void)(cb), (void)(mask))
#define gpio_add_callback(port, cb) ((void)(port), (void)(cb))
#define gpio_pin_get_dt(pin) ((void)(pin), ++pin_reads, 1)
#include "production.inc"

static void snapshot_and_button(void) {
    for (unsigned marker = 0; marker < 256; marker++) {
        for (unsigned vbus = 0; vbus < 2; vbus++) {
            uint32_t raw = 0x100u | (vbus ? POWER_RESETREAS_VBUS_Msk : 0);
            power_regs.GPREGRET = marker;
            power_regs.GPREGRET2 = marker ^ 0xA5u;
            timer_regs.CC[0] = 0x12345678;
            timer_regs.CC[1] = capability_token;
            RESET_REGISTER = raw;
            assert(sys_reset_reason_init() == 0);
            /* A plain host register records the exact W1C write, not its HW effect. */
            assert(RESET_REGISTER == raw);
            assert(timer_regs.CC[0] == 0x12345678);
#ifdef TEST_UF2
            assert(timer_regs.CC[1] == 0);
#else
            assert(timer_regs.CC[1] == capability_token);
#endif
            assert(power_regs.GPREGRET2 == (marker ^ 0xA5u));
            /* Later ESB timer writes must neither grant nor revoke support. */
            timer_regs.CC[1] = recovery_capable ? 0 : 0x52435631u;
            uint32_t expected = raw;
#ifdef TEST_UF2
            if (recovery_capable && (marker == 0xE1 || marker == 0xE2))
                expected |= POWER_RESETREAS_DOG_Msk;
#endif
            assert(boot_reset_reason == expected && power_regs.GPREGRET == marker);
            RESET_REGISTER = 0; /* W1C hardware effect before the later button init. */
            button_held_from_init = false;
            pin_reads = 0;
            assert(sys_button_init() == 0);
            assert(button_held_from_init == !vbus && pin_reads == !vbus);
        }
    }
}
static void markers_and_uptime(void) {
    for (unsigned marker = 0; marker < 256; marker++) {
        power_regs.GPREGRET = marker;
        power_regs.GPREGRET2 = marker ^ 0xA5u;
        sys_skip_dfu_marker();
#ifdef TEST_UF2
        assert(power_regs.GPREGRET == (marker ? marker : 0x6D));
#else
        assert(power_regs.GPREGRET == marker);
#endif
        const int64_t starts[] = {0, 59999, 60000, 90000};
        for (unsigned start = 0; start < sizeof(starts)/sizeof(starts[0]); start++) {
        power_regs.GPREGRET = marker;
        now = starts[start];
        confirms = sleeps = led_calls = 0;
        sleep_marker = marker;
        assert(receiver_main() == 0);
        assert(locks == 0);
#ifdef TEST_LEGACY
        assert(now == starts[start] && sleeps == 0);
#else
        assert(now == (starts[start] < 60000 ? 60000 : starts[start]));
        assert(sleeps == (starts[start] < 60000));
#endif
#ifdef TEST_UF2
        assert(power_regs.GPREGRET ==
               ((recovery_capable && (marker == 0xE1 || marker == 0xE2)) ? 0 : marker));
#else
        assert(power_regs.GPREGRET == marker);
#endif
        assert(power_regs.GPREGRET2 == (marker ^ 0xA5u));
#ifdef TEST_MCUBOOT
        assert(confirms == 1);
#else
        assert(confirms == 0);
#endif
        }
    }
#ifdef TEST_MCUBOOT
    confirmed = true;
    confirms = 0;
    now = 60000;
    assert(receiver_main() == 0 && confirms == 0);
    confirmed = false;
    confirm_error = -1;
    confirms = 0;
    assert(receiver_main() == 0 && confirms == 1);
#endif
#ifdef TEST_UF2
    const uint32_t pending[] = {0x57, 0xA8, 0x4E, 0xE3};
    for (unsigned i = 0; i < sizeof(pending)/sizeof(pending[0]); i++) {
        power_regs.GPREGRET = 0xE2;
        if (recovery_capable) {
            inject_marker = pending[i];
        } else {
            power_regs.GPREGRET = pending[i];
        }
        sys_mark_boot_success();
        assert(power_regs.GPREGRET == pending[i] && locks == 0);
        assert(inject_marker == 0);
    }
#endif
}
static void fatal(void) {
    for (unsigned marker = 0; marker < 256; marker++) {
        power_regs.GPREGRET = marker;
        power_regs.GPREGRET2 = marker ^ 0xA5u;
        barriers = panics = logs = reboots = 0;
        if (!setjmp(reboot_exit)) k_sys_fatal_error_handler(42, NULL);
        assert(reboots == 1 && locks == 0);
#ifdef TEST_UF2
        assert(power_regs.GPREGRET == (recovery_capable ? 0xE3u : 0x57u));
        assert(barriers == 1 && panics == 0 && logs == 0);
#else
        assert(power_regs.GPREGRET == marker && barriers == 0 && panics == 1 && logs == 1);
#endif
        assert(power_regs.GPREGRET2 == (marker ^ 0xA5u));
    }
}
int main(void) {
    /* Fatal before PRE_KERNEL_1 must use legacy fallback, even with a token. */
    timer_regs.CC[1] = 0x52435631u;
    fatal();
    const uint32_t tokens[] = {
        0, 0x52435630u, 0x52435632u, 0xFFFFFFFFu, 0x00005631u, 0x52435631u, 0
    };
    for (unsigned i = 0; i < sizeof(tokens)/sizeof(tokens[0]); i++) {
        capability_token = tokens[i];
#ifdef TEST_UF2
        recovery_capable = capability_token == 0x52435631u;
#endif
        snapshot_and_button();
        markers_and_uptime();
        fatal();
    }
    puts("receiver recovery: runtime capability, timer reuse, snapshot/button, "
         "60s acknowledgment, pending requests, early/final fatal passed");
    return 0;
}
