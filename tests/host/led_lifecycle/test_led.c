/* Deterministic UP scheduling around unchanged production LED/driver bodies.
 * No RF, physical timing, or Zephyr scheduler implementation is modeled. */
#define _XOPEN_SOURCE 700
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include "system/led.h"
#include "thread_priority.h"

#define LED_STRIP_EXISTS (VARIANT == 3)
#define LED_EXISTS (VARIANT == 1 || VARIANT == 2)
#define LED0_EXISTS 0
#define LED1_EXISTS 0
#define LED2_EXISTS 0
#define LED3_EXISTS 0
#define PWM_LED_EXISTS (VARIANT == 2)
#define PWM_LED1_EXISTS 0
#define PWM_LED2_EXISTS 0
#define CONFIG_LED_DEFAULT_COLOR_R 10000
#define CONFIG_LED_DEFAULT_COLOR_G 10000
#define CONFIG_LED_DEFAULT_COLOR_B 10000
#define SPI_FRAME_BITS 8
#define BITS_PER_COLOR_CHANNEL 8
#define WS2812_SPI_ALL_SYMBOLS_ARE_8_BITS 1
#define BIT(n) (1U << (n))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1U) / (d))
#define K_NO_WAIT 0
#define K_FOREVER (-1)
#define K_MSEC(n) (n)
#define GPIO_OUTPUT 1
#define GPIO_DISCONNECTED 0
#define SYS_INIT(...)
#define ARG_UNUSED(x) ((void)(x))
#define LOG_DBG(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) (logged_errors++)
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "CHECK failed %s:%d: %s\n", __FILE__, __LINE__, #x); exit(2); } } while (0)

struct sim_thread;
struct k_spinlock { bool held; };
typedef int k_spinlock_key_t;
struct k_sem { unsigned count, limit; struct sim_thread *waiter; };
#define K_SEM_DEFINE(name, initial, maximum) struct k_sem name = {.count = initial, .limit = maximum}
struct k_mutex { struct sim_thread *owner; };
struct spi_dt_spec { int unused; };
struct spi_buf { void *buf; size_t len; };
struct spi_buf_set { const struct spi_buf *buffers; size_t count; };
struct device { const char *name; const void *config; void *data; const void *api; };
struct gpio_dt_spec { int unused; };
struct pwm_dt_spec { int period; };
static const struct gpio_dt_spec led = {0};
static const struct pwm_dt_spec pwm_led = {.period = 10000};

enum sim_event { EV_NONE, EV_LOCK_HELD, EV_USLEEP, EV_MSLEEP, EV_SEM_WAIT,
                 EV_AFTER_DRAIN, EV_BEFORE_WAIT, EV_DONE };
struct sim_thread {
    ucontext_t ctx;
    unsigned char stack[65536];
    bool sleeping, done;
    int sleep_ms, wait_ms, sem_result;
    struct k_sem *waiting;
};
static struct sim_thread worker, caller;
static struct sim_thread *current;
static struct sim_thread *const led_thread_id = &worker;
static ucontext_t host;
static enum sim_event event;
static bool isr, pause_after_lock, pause_after_drain, pause_before_wait;
static int64_t now_ms;
static unsigned logged_errors, spi_calls, black_frames, gpio_calls, pwm_calls;
static bool last_frame_black, gpio_connected;
static bool spi_failure, gpio_disconnect_failure;
static enum sys_led_pattern requested_pattern;
static int requested_priority;

static void yield_event(enum sim_event why);
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock);
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key);
static void k_sem_give(struct k_sem *sem);
static int k_sem_take(struct k_sem *sem, int timeout);
static int k_mutex_lock(struct k_mutex *mutex, int timeout);
static int k_mutex_unlock(struct k_mutex *mutex);
static struct sim_thread *k_current_get(void);
static bool k_is_in_isr(void);
static int64_t k_uptime_get(void);
static int32_t k_usleep(int32_t us);
static int32_t k_msleep(int32_t ms);
static int spi_write_dt(const struct spi_dt_spec *bus, const struct spi_buf_set *tx);
static bool device_is_ready(const struct device *dev);
static int gpio_pin_configure_dt(const struct gpio_dt_spec *gpio, int flags);
static int gpio_pin_set_dt(const struct gpio_dt_spec *gpio, int value);
static int pwm_set_pulse_dt(const struct pwm_dt_spec *pwm, int pulse);

#include "driver.inc"
#include "production.inc"

static void yield_event(enum sim_event why)
{
    CHECK(current != NULL);
    event = why;
    CHECK(swapcontext(&current->ctx, &host) == 0);
}
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
    CHECK(!lock->held);
    lock->held = true;
    return 0;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    (void)key;
    CHECK(lock->held);
    lock->held = false;
}
static void k_sem_give(struct k_sem *sem)
{
    if (sem->waiter) {
        struct sim_thread *waiter = sem->waiter;
        sem->waiter = NULL;
        waiter->waiting = NULL;
        waiter->sem_result = 0;
    } else if (sem->count < sem->limit) {
        sem->count++;
    }
}
static int k_sem_take(struct k_sem *sem, int timeout)
{
    if (current == &worker && timeout != K_NO_WAIT && pause_before_wait) {
        pause_before_wait = false;
        yield_event(EV_BEFORE_WAIT);
    }
    int result = -EAGAIN;
    if (sem->count) {
        sem->count--;
        result = 0;
    } else if (timeout != K_NO_WAIT) {
        CHECK(current != NULL && sem->waiter == NULL);
        sem->waiter = current;
        current->waiting = sem;
        current->wait_ms = timeout;
        current->sem_result = -EAGAIN;
        yield_event(EV_SEM_WAIT);
        CHECK(current->waiting == NULL);
        result = current->sem_result;
    }
    if (current == &worker && timeout == K_NO_WAIT && pause_after_drain) {
        pause_after_drain = false;
        yield_event(EV_AFTER_DRAIN);
    }
    return result;
}
static int k_mutex_lock(struct k_mutex *mutex, int timeout)
{
    CHECK(current != NULL && timeout == K_FOREVER && mutex->owner == NULL);
    mutex->owner = current;
    if (current == &worker && pause_after_lock) {
        pause_after_lock = false;
        yield_event(EV_LOCK_HELD);
    }
    return 0;
}
static int k_mutex_unlock(struct k_mutex *mutex)
{
    CHECK(mutex->owner == current);
    mutex->owner = NULL;
    return 0;
}
static struct sim_thread *k_current_get(void) { return current; }
static bool k_is_in_isr(void) { return isr; }
static int64_t k_uptime_get(void) { return now_ms; }
static int32_t k_usleep(int32_t us)
{
    CHECK(current != NULL && us > 0);
    current->sleeping = true;
    current->sleep_ms = (us + 999) / 1000;
    yield_event(EV_USLEEP);
    return 0;
}
static int32_t k_msleep(int32_t ms)
{
    CHECK(current != NULL && ms > 0);
    current->sleeping = true;
    current->sleep_ms = ms;
    yield_event(EV_MSLEEP);
    return 0;
}
static int spi_write_dt(const struct spi_dt_spec *bus, const struct spi_buf_set *tx)
{
    (void)bus;
    CHECK(current == &worker && tx->count == 1 && tx->buffers[0].len == 24);
#if LED_STRIP_EXISTS
    const uint8_t *bytes = tx->buffers[0].buf;
    last_frame_black = true;
    for (size_t i = 0; i < tx->buffers[0].len; i++) {
        if (bytes[i] != wsconfig.zero_frame) last_frame_black = false;
    }
#endif
    spi_calls++;
    if (last_frame_black) black_frames++;
    return spi_failure ? -EIO : 0;
}
static bool device_is_ready(const struct device *dev) { return dev != NULL; }
static int gpio_pin_configure_dt(const struct gpio_dt_spec *gpio, int flags)
{
    (void)gpio;
    CHECK(current == &worker);
    gpio_calls++;
    if (flags == GPIO_DISCONNECTED && gpio_disconnect_failure) return -EIO;
    gpio_connected = flags != GPIO_DISCONNECTED;
    return 0;
}
static int gpio_pin_set_dt(const struct gpio_dt_spec *gpio, int value)
{
    (void)gpio; (void)value;
    CHECK(current == &worker);
    gpio_calls++;
    return 0;
}
static int pwm_set_pulse_dt(const struct pwm_dt_spec *pwm, int pulse)
{
    (void)pwm; (void)pulse;
    CHECK(current == &worker);
    pwm_calls++;
    return 0;
}

static enum sim_event run_thread(struct sim_thread *thread)
{
    CHECK(current == NULL && !thread->done && !thread->sleeping && thread->waiting == NULL);
    event = EV_NONE;
    current = thread;
    CHECK(swapcontext(&host, &thread->ctx) == 0);
    current = NULL;
    CHECK(event != EV_NONE);
    return event;
}
static void expire_sleep(struct sim_thread *thread)
{
    CHECK(thread->sleeping);
    now_ms += thread->sleep_ms;
    thread->sleeping = false;
}
static void expire_sem(struct sim_thread *thread)
{
    CHECK(thread->waiting && thread->wait_ms >= 0);
    now_ms += thread->wait_ms;
    thread->waiting->waiter = NULL;
    thread->waiting = NULL;
    thread->sem_result = -EAGAIN;
}
static void worker_entry(void) { led_thread(); worker.done = true; yield_event(EV_DONE); abort(); }
static void caller_entry(void)
{
    set_led(requested_pattern, requested_priority);
    caller.done = true;
    yield_event(EV_DONE);
    abort();
}
static void init_context(struct sim_thread *thread, void (*entry)(void))
{
    CHECK(getcontext(&thread->ctx) == 0);
    thread->ctx.uc_stack.ss_sp = thread->stack;
    thread->ctx.uc_stack.ss_size = sizeof(thread->stack);
    thread->ctx.uc_link = &host;
    makecontext(&thread->ctx, entry, 0);
}
static void init_case(enum sys_led_pattern request, int priority)
{
    memset(&worker, 0, sizeof(worker));
    memset(&caller, 0, sizeof(caller));
#if LED_STRIP_EXISTS
    memset(&wsdata, 0, sizeof(wsdata));
#endif
    current = NULL;
    isr = pause_after_lock = pause_after_drain = pause_before_wait = false;
    now_ms = 0;
    logged_errors = spi_calls = black_frames = gpio_calls = pwm_calls = 0;
    last_frame_black = gpio_connected = false;
    spi_failure = gpio_disconnect_failure = false;
    requested_pattern = request;
    requested_priority = priority;
#if LED_EXISTS || LED_STRIP_EXISTS
    memset(&led_state_lock, 0, sizeof(led_state_lock));
    led_changed.count = 0;
    led_changed.waiter = NULL;
    for (int i = 0; i < SYS_LED_PATTERN_DEPTH; i++) led_patterns[i] = SYS_LED_PATTERN_OFF;
    led_patterns[SYS_LED_PRIORITY_SYSTEM] = SYS_LED_PATTERN_LONG;
    current_led_pattern = SYS_LED_PATTERN_LONG;
    current_priority = SYS_LED_PRIORITY_SYSTEM;
    led_generation = 1;
    led_off_generation = 0;
    led_pattern_state = 0;
#endif
    init_context(&worker, worker_entry);
    init_context(&caller, caller_entry);
}

#if LED_EXISTS || LED_STRIP_EXISTS
static void run_worker_to_wait(void)
{
    for (int steps = 0; steps < 40; steps++) {
        if (worker.waiting) return;
        if (worker.sleeping) expire_sleep(&worker);
        enum sim_event why = run_thread(&worker);
        CHECK(why == EV_USLEEP || why == EV_SEM_WAIT);
    }
    CHECK(false);
}
static void test_priority_and_oneshot_guard(void)
{
    init_case(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SYSTEM);
    current = &caller;
    isr = true;
    set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
    CHECK(current_led_pattern == SYS_LED_PATTERN_SHORT && current_priority == SYS_LED_PRIORITY_CONNECTION);
    set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_BOOT);
    uint32_t obsolete = led_generation;
    set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_BOOT);
    led_finish_oneshot(obsolete, SYS_LED_PRIORITY_BOOT, SYS_LED_PATTERN_OFF);
    CHECK(current_led_pattern == SYS_LED_PATTERN_ON && led_patterns[SYS_LED_PRIORITY_BOOT] == SYS_LED_PATTERN_ON);
    set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_BOOT);
    CHECK(current_led_pattern == SYS_LED_PATTERN_SHORT);
    set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_BOOT);
    set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CONNECTION);
    set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SYSTEM);
    CHECK(current_led_pattern == SYS_LED_PATTERN_OFF_FORCE);
    set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_BOOT);
    CHECK(current_led_pattern == SYS_LED_PATTERN_ON && current_priority == SYS_LED_PRIORITY_SYSTEM);
    CHECK(spi_calls == 0 && gpio_calls == 0 && pwm_calls == 0);
    current = NULL;
    isr = false;
}
static void test_oneshot_completion(enum sys_led_pattern oneshot, bool force)
{
    init_case(oneshot, SYS_LED_PRIORITY_BOOT);
    CHECK(run_thread(&caller) == EV_DONE);
    for (int steps = 0; steps < 210; steps++) {
        run_worker_to_wait();
        if (force && current_led_pattern == SYS_LED_PATTERN_OFF_FORCE &&
            led_off_generation == led_generation) {
#if LED_STRIP_EXISTS
            CHECK(last_frame_black);
#else
            CHECK(!gpio_connected);
#endif
            return;
        }
        if (!force && current_led_pattern == SYS_LED_PATTERN_LONG) {
            CHECK(current_priority == SYS_LED_PRIORITY_SYSTEM);
            CHECK(led_patterns[SYS_LED_PRIORITY_BOOT] == SYS_LED_PATTERN_OFF);
            return;
        }
        CHECK(worker.waiting && worker.wait_ms >= 0);
        expire_sem(&worker);
    }
    CHECK(false);
}

static void test_lost_wakeup(bool after_drain)
{
    init_case(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
    pause_after_drain = after_drain;
    pause_before_wait = !after_drain;
    for (int steps = 0; ; steps++) {
        CHECK(steps < 20);
        if (worker.sleeping) expire_sleep(&worker);
        enum sim_event why = run_thread(&worker);
        if (why == (after_drain ? EV_AFTER_DRAIN : EV_BEFORE_WAIT)) break;
        CHECK(why == EV_USLEEP);
    }
    CHECK(run_thread(&caller) == EV_DONE);
    run_worker_to_wait();
    CHECK(current_led_pattern == SYS_LED_PATTERN_SHORT && led_pattern_state == 1);
    CHECK(worker.wait_ms == 100 && logged_errors == 0);
}
static void test_wake_sleeping_worker(void)
{
    init_case(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
    run_worker_to_wait();
    CHECK(worker.waiting && worker.wait_ms == 500);
    CHECK(run_thread(&caller) == EV_DONE && worker.waiting == NULL);
    run_worker_to_wait();
    CHECK(current_led_pattern == SYS_LED_PATTERN_SHORT && worker.wait_ms == 100);
}

static void test_thread_force_off(void)
{
    init_case(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_BOOT);
    CHECK(run_thread(&caller) == EV_MSLEEP && !caller.done);
    run_worker_to_wait();
    CHECK(led_off_generation == led_generation);
#if LED_STRIP_EXISTS
    CHECK(last_frame_black);
#else
    CHECK(!gpio_connected);
#endif
    expire_sleep(&caller);
    CHECK(run_thread(&caller) == EV_DONE && logged_errors == 0);
}

static void test_isr_force_off(void)
{
    init_case(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_BOOT);
    isr = true;
    CHECK(run_thread(&caller) == EV_DONE);
    isr = false;
    CHECK(spi_calls == 0 && gpio_calls == 0 && pwm_calls == 0);
    run_worker_to_wait();
    CHECK(current_led_pattern == SYS_LED_PATTERN_OFF_FORCE && led_off_generation == led_generation);
#if LED_STRIP_EXISTS
    CHECK(last_frame_black);
#else
    CHECK(!gpio_connected);
#endif
}
static void test_failed_black_is_not_acknowledged(void)
{
    init_case(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_BOOT);
    spi_failure = gpio_disconnect_failure = true;
    CHECK(run_thread(&caller) == EV_MSLEEP);
    run_worker_to_wait();
    CHECK(led_off_generation != led_generation);
    for (int steps = 0; ; steps++) {
        CHECK(steps <= LED_OFF_WAIT_MS + 1);
        expire_sleep(&caller);
        enum sim_event why = run_thread(&caller);
        if (why == EV_DONE) break;
        CHECK(why == EV_MSLEEP);
    }
    CHECK(logged_errors >= 2 && led_off_generation != led_generation);
}
static void test_superseded_force_off(void)
{
    init_case(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_BOOT);
    CHECK(run_thread(&caller) == EV_MSLEEP);
    current = &caller; /* ISR interrupts the sleeping caller in the model. */
    isr = true;
    set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_BOOT);
    isr = false;
    current = NULL;
    expire_sleep(&caller);
    CHECK(run_thread(&caller) == EV_DONE);
    CHECK(current_led_pattern == SYS_LED_PATTERN_ON && logged_errors == 1);
}


static void test_bounded_force_off(void)
{
    init_case(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_BOOT);
    for (int steps = 0; ; steps++) {
        CHECK(steps <= LED_OFF_WAIT_MS + 1);
        enum sim_event why = run_thread(&caller);
        if (why == EV_DONE) break;
        CHECK(why == EV_MSLEEP);
        expire_sleep(&caller);
    }
    CHECK(now_ms == LED_OFF_WAIT_MS && logged_errors == 1 && !worker.done);
}
#endif

#if LED_STRIP_EXISTS
static void test_old_suspended_owner_interleaving(bool at_reset, int caller_priority)
{
    /* Forced schedules: a higher-priority caller can preempt at the lock;
     * an equal-priority caller can also run while the owner is sleeping. */
    CHECK(caller_priority <= LED_THREAD_PRIORITY);
    CHECK(at_reset || caller_priority < LED_THREAD_PRIORITY);
    init_case(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
    pause_after_lock = !at_reset;
    CHECK(run_thread(&worker) == (at_reset ? EV_USLEEP : EV_LOCK_HELD));
    CHECK(wsdata.lock.owner == &worker);
    CHECK(run_thread(&caller) == EV_DONE);
    CHECK(caller.done && wsdata.lock.owner == &worker);
    run_worker_to_wait();
    CHECK(wsdata.lock.owner == NULL && current_led_pattern == SYS_LED_PATTERN_SHORT);
    CHECK(worker.wait_ms == 100 && led_pattern_state == 1 && logged_errors == 0);
}
static void test_force_off_waits_for_black(bool at_reset)
{
    init_case(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_BOOT);
    pause_after_lock = !at_reset;
    CHECK(run_thread(&worker) == (at_reset ? EV_USLEEP : EV_LOCK_HELD));
    CHECK(wsdata.lock.owner == &worker);
    CHECK(run_thread(&caller) == EV_MSLEEP);
    CHECK(!caller.done && wsdata.lock.owner == &worker);
    run_worker_to_wait();
    CHECK(wsdata.lock.owner == NULL && led_off_generation == led_generation);
    CHECK(last_frame_black && black_frames > 0);
    expire_sleep(&caller);
    CHECK(run_thread(&caller) == EV_DONE);
    CHECK(last_frame_black && logged_errors == 0);
}
#endif

int main(void)
{
#if LED_EXISTS || LED_STRIP_EXISTS
    test_priority_and_oneshot_guard();
    test_oneshot_completion(SYS_LED_PATTERN_ONESHOT_PROGRESS, false);
    test_oneshot_completion(SYS_LED_PATTERN_ONESHOT_POWEROFF, true);
    test_lost_wakeup(true);
    test_lost_wakeup(false);
    test_wake_sleeping_worker();
    test_thread_force_off();
    test_isr_force_off();
    test_failed_black_is_not_acknowledged();
    test_superseded_force_off();
    test_bounded_force_off();
#if LED_STRIP_EXISTS
    test_old_suspended_owner_interleaving(false, ESB_THREAD_PRIORITY);
    test_old_suspended_owner_interleaving(false, 0);
    test_old_suspended_owner_interleaving(true, ESB_THREAD_PRIORITY);
    test_old_suspended_owner_interleaving(true, STATUS_THREAD_PRIORITY);
    test_old_suspended_owner_interleaving(true, 0);
    test_force_off_waits_for_black(false);
    test_force_off_waits_for_black(true);
#endif
#else
    init_case(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_BOOT);
    CHECK(run_thread(&caller) == EV_DONE);
    CHECK(run_thread(&worker) == EV_DONE);
#endif
    printf("LED behavioral replay variant %d passed\n", VARIANT);
    return 0;
}
