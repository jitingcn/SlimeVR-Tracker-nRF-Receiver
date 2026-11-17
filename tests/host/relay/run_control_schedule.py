#!/usr/bin/env python3
"""Run the receiver's control scheduler against deterministic RTC failures.

The receiver helpers, planner, callbacks and control thread are extracted from
esb.c. Only the RTC, ESB and kernel boundaries are simulated; geometry,
timeline, demand and coordinator policy are the production implementations.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent / 'harness'))
from c_extract import extract_block
from relay_config import select_relay

source = select_relay((ROOT / 'src/connection/esb.c').read_text())


def function(name):
    return extract_block(
        source,
        r'^(?:__attribute__\([^\n]*\)\s+)?static\s+(?:RELAY_RAMFUNC\s+)?'
        r'(?:void|bool|int|int32_t|uint8_t)\s+' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{',
    )


def structure(name):
    return extract_block(source, rf'^struct {name}\s*\{{', semicolon=True)


constants = '\n'.join(re.findall(
    r'^#define (?:RELAY_CONTROL_CALLBACK_LEAD_TICKS|RELAY_RTC_MIN_LEAD_TICKS)\b[^\n]*',
    source, re.M))
if len(constants.splitlines()) != 2:
    raise ValueError('control compare constants not found')

prefix = r'''
#include <assert.h>
#include <errno.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CONFIG_RELAY_RUNTIME_LAYOUT 1
#define __ramfunc
#define ARG_UNUSED(x) ((void)(x))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define __DMB() ((void)0)
#define IRQ_PROBE_BEGIN(name)
#define IRQ_PROBE_RESTART(name)
#define IRQ_PROBE_END(name)
#define LOG_ERR(...) ((void)0)
#define K_FOREVER 0
#define K_TIMEOUT_ABS_TICKS(ticks) (ticks)
#include "relay_coord.h"
#include "relay_demand.h"

struct esb_payload { uint8_t length, pipe; bool noack; uint8_t data[72]; };
static struct relay_geometry geometry;
const struct relay_geometry *relay_geometry_active = &geometry;
struct relay_timeline relay_timeline_active;
static struct relay_demand relay_demand_state;
static struct relay_runtime_settings relay_applied_settings;
static struct relay_coord relay_coord;
static struct relay_control relay_grants;
static bool esb_initialized;
static uint8_t relay_profile, relay_epoch;
static uint32_t tdma_active_mask, relay_origin, relay_session, relay_layout_generation;
static uint32_t relay_frame_anchor, relay_start_anchor;
static uint32_t relay_control_repeat_sent, relay_control_repeat_failed;
static int relay_control_repeat_err;
static uint32_t relay_control_sent, relay_control_failed;
static uint32_t relay_control_late, relay_control_plan_max, relay_control_grant_max;
static uint32_t relay_control_entry_late, relay_control_entry_max;
static int relay_control_err;
static int radio_operation_busy;
static bool relay_control_ready;
static int relay_freeze_done;
'''
prefix += '\n' + structure('relay_control_diagnostics')
prefix += '\nstatic struct relay_control_diagnostics relay_control_diag;\n'
prefix += structure('relay_control_work') + '\n'
prefix += constants + r'''

typedef void (*z_nrf_rtc_timer_compare_handler_t)(int32_t, uint64_t, void *);
struct compare_slot {
    bool armed;
    uint64_t tick;
    z_nrf_rtc_timer_compare_handler_t callback;
    void *context;
};
static struct compare_slot compares[2];
static struct { uint32_t COUNTER; } rtc;
#define NRF_RTC1 (&rtc)
static uint64_t clock_ticks;
static unsigned exact_calls, aborts, fail_on_exact, advance_after_launch, freeze_lag, arm_work;
static int injected_error;
static unsigned irq_depth, sleeps, prepare_count;
static uint32_t prepared_frames[16];
static bool inject_late_prepare, check_future_preparation;
static bool fail_prepare, fail_freeze;
static int fail_begin, fail_arm, fail_wait;
static unsigned change_after_failure;
static uint32_t original_frame, original_start;
static jmp_buf stop_thread;
static uint8_t dma[72];

static unsigned irq_lock(void) { return irq_depth++; }
static void irq_unlock(unsigned key) { assert(irq_depth == key + 1); irq_depth = key; }
static bool atomic_cas(int *p, int old, int value)
{ if (*p != old) return false; *p = value; return true; }
static void atomic_set(int *p, int value) { *p = value; }
static uint64_t k_uptime_ticks(void) { return clock_ticks; }
static uint32_t k_uptime_get_32(void) { return (uint32_t)(clock_ticks * 1000U / 32768U); }
static uint32_t k_ticks_to_us_floor32(uint32_t ticks) { return (uint64_t)ticks * 1000000U / 32768U; }
static uint32_t k_ticks_to_us_ceil32(uint32_t ticks) { return ((uint64_t)ticks * 1000000U + 32767U) / 32768U; }
static void k_usleep(uint32_t us)
{
    assert(!irq_depth);
    if (relay_control_sent) longjmp(stop_thread, 1);
    if (change_after_failure && prepare_count == 1) {
        assert(!compares[0].armed && !compares[1].armed);
        if (change_after_failure == 1) ++relay_session;
        else ++relay_layout_generation;
        change_after_failure = 0;
    }
    assert(++sleeps < 1024);
    uint32_t ticks = ((uint64_t)us * 32768U + 999999U) / 1000000U;
    clock_ticks += ticks ? ticks : 1U;
}
static int32_t z_nrf_rtc_timer_chan_alloc(void)
{ static int next; int channel = next++ % 2; return channel; }
static void z_nrf_rtc_timer_chan_free(int32_t channel) { (void)channel; }
static int z_nrf_rtc_timer_compare_int_lock(int32_t channel) { (void)channel; return 0; }
static uint32_t z_nrf_rtc_timer_compare_evt_address_get(int32_t channel) { (void)channel; return 1; }
static uint64_t z_nrf_rtc_timer_get_ticks(uint64_t ticks) { return ticks; }
static uint64_t z_nrf_rtc_timer_read(void) { return clock_ticks; }
static int z_nrf_rtc_timer_exact_set(int32_t channel, uint64_t tick,
                                     z_nrf_rtc_timer_compare_handler_t cb, void *context)
{
    assert(channel == 0 || channel == 1);
    ++exact_calls;
    /* Even a partially armed, failing device must be cleaned up by the caller. */
    compares[channel] = (struct compare_slot){true, tick, cb, context};
    if (exact_calls == fail_on_exact) return injected_error;
    if (tick <= clock_ticks || tick - clock_ticks < 3U) return -EINVAL;
    if (channel == 0) clock_ticks += advance_after_launch;
    return 0;
}
static void z_nrf_rtc_timer_abort(int32_t channel)
{
    assert(channel == 0 || channel == 1);
    ++aborts;
    compares[channel] = (struct compare_slot){0};
}
static uint8_t *esb_prx_tx_prepare(struct esb_payload *payload)
{
    assert(prepare_count < sizeof(prepared_frames) / sizeof(prepared_frames[0]));
    uint32_t frame = relay_get_be32(payload->data + RELAY_CONTROL_OFF_FRAME);
    prepared_frames[prepare_count++] = frame;
    memcpy(dma, payload->data, sizeof(dma));
    if (check_future_preparation && frame == original_frame + 1U) {
        /* Selection/planning happens while the old frame still owns RX and demand. */
        assert(relay_timeline_active.frame == original_frame);
        assert(relay_demand_state.frame == original_frame);
        uint32_t tick = original_start + RELAY_REPAIR_START + 1U;
        assert(relay_frame_near(original_frame, original_start, tick) == original_frame);
        assert(relay_frame_phase(relay_origin, original_frame, tick) == RELAY_REPAIR_START + 1U);
        check_future_preparation = false;
    }
    if (inject_late_prepare) {
        inject_late_prepare = false;
        clock_ticks += 13U;
    }
    if (fail_prepare) {
        fail_prepare = false;
        return NULL;
    }
    return dma;
}
static int esb_prx_tx_begin(void)
{ int err = fail_begin; fail_begin = 0; return err; }
static int esb_prx_tx_arm_event(uint32_t delay, uint32_t event)
{
    assert(delay && event && compares[0].armed);
    clock_ticks += arm_work;
    rtc.COUNTER = (uint32_t)clock_ticks & 0x00ffffffU;
    int err = fail_arm; fail_arm = 0;
    return err ? err : clock_ticks >= compares[0].tick ? -ETIME : 0;
}
static int esb_prx_tx_wait(uint32_t timeout)
{
    assert(timeout && compares[0].armed);
    clock_ticks = compares[0].tick + 1U;
    compares[0] = (struct compare_slot){0};
    int err = fail_wait;
    fail_wait = 0;
    return err;
}
static void k_sem_give(int *sem) { assert(sem == &relay_freeze_done); ++*sem; }
static void k_sem_take(int *sem, int timeout)
{
    assert(sem == &relay_freeze_done && timeout == K_FOREVER && !irq_depth);
    assert(compares[1].armed && compares[1].callback && compares[1].context);
    struct compare_slot callback = compares[1];
    compares[1] = (struct compare_slot){0};
    struct relay_control_work *work = callback.context;
    clock_ticks = work->freeze_rtc + (fail_freeze ? -1 : (int)freeze_lag);
    rtc.COUNTER = (uint32_t)clock_ticks & 0x00ffffffU;
    fail_freeze = false;
    callback.callback(1, callback.tick, callback.context);
    assert(*sem == 1);
    --*sem;
}
'''

names = (
    'relay_frame_repairs', 'relay_frame_announce',
    'relay_launch_margin', 'relay_arm_failure',
    'relay_freeze_demand', 'relay_freeze_arm', 'relay_repeat_encode', 'relay_repeat_arm',
    'relay_freeze_callback', 'relay_repeat_callback',
    'relay_control_target', 'relay_control_compare_ready', 'relay_control_cancel', 'relay_control_schedule',
    'relay_control_plan', 'relay_control_eligible', 'relay_control_context_current',
    'relay_control_thread',
)
production = '\n\n'.join(function(name) for name in names)

cases = r'''
static void reset_device(void)
{
    memset(compares, 0, sizeof(compares));
    memset(&relay_control_diag, 0, sizeof(relay_control_diag));
    exact_calls = aborts = fail_on_exact = advance_after_launch = freeze_lag = arm_work = 0;
    injected_error = -EIO;
    relay_control_sent = relay_control_failed = 0;
    relay_control_repeat_sent = relay_control_repeat_failed = 0;
    relay_control_repeat_err = relay_control_err = 0;
    relay_control_late = relay_control_plan_max = relay_control_grant_max = 0;
    relay_control_entry_late = relay_control_entry_max = 0;
    relay_freeze_done = radio_operation_busy = 0;
    irq_depth = sleeps = prepare_count = 0;
    inject_late_prepare = check_future_preparation = false;
    fail_prepare = fail_freeze = false;
    fail_begin = fail_arm = fail_wait = 0;
    change_after_failure = 0;
}

static void setup_geometry(uint32_t frame, uint32_t start, uint8_t direct, uint8_t repairs)
{
    assert(relay_geometry_make(&geometry, direct, repairs, 16, 120));
    relay_applied_settings = (struct relay_runtime_settings){16, 5, 16, 100, 120, 0, 0};
    relay_demand_reset(&relay_demand_state, frame, 1);
    relay_demand_state.queued[0] = repairs;
    relay_demand_state.queued[1] = repairs;
    relay_demand_state.queued[2] = repairs;
    relay_demand_state.target = repairs;
    relay_timeline_clear(&relay_timeline_active);
    relay_frame_announce(frame, start);
    relay_origin = start;
    relay_frame_anchor = frame;
    relay_start_anchor = start;
}

static void selection(uint32_t frame, uint32_t start, uint8_t direct, uint8_t repairs)
{
    setup_geometry(frame, start, direct, repairs);
    uint32_t next_start, output_frame, preamble;
    assert(relay_timeline_start(&relay_timeline_active, frame + 1U, &next_start));
    assert(next_start - start == relay_runtime_period(frame, direct, repairs, 16, 120, 0));
    struct relay_timeline before;
    struct relay_demand demand;
    memcpy(&before, &relay_timeline_active, sizeof(before));
    memcpy(&demand, &relay_demand_state, sizeof(demand));
    uint32_t current_preamble = start + RELAY_CONTROL_PREAMBLE;
    uint32_t next_preamble = next_start + RELAY_CONTROL_PREAMBLE;

    assert(relay_control_target(start, UINT32_MAX, false, &output_frame, &preamble));
    assert(output_frame == frame && preamble == current_preamble);
    assert(relay_control_target(start, frame - 1U, true, &output_frame, &preamble));
    assert(output_frame == frame && preamble == current_preamble);
    assert(relay_control_target(start, frame, true, &output_frame, &preamble));
    assert(output_frame == frame + 1U && preamble == next_preamble);
    assert(!relay_control_target(start, frame + 1U, true, &output_frame, &preamble));
    assert(relay_control_target(current_preamble - 12U, 0, false, &output_frame, &preamble));
    assert(output_frame == frame && preamble == current_preamble);
    assert(relay_control_target(current_preamble - 11U, 0, false, &output_frame, &preamble));
    assert(output_frame == frame + 1U && preamble == next_preamble);
    assert(relay_control_target(current_preamble + 1U, 0, false, &output_frame, &preamble));
    assert(output_frame == frame + 1U && preamble == next_preamble);
    assert(!memcmp(&before, &relay_timeline_active, sizeof(before)));
    assert(!memcmp(&demand, &relay_demand_state, sizeof(demand)));
    uint32_t old_repair = start + RELAY_REPAIR_START + 1U;
    assert(relay_frame_near(frame, start, old_repair) == frame);
    assert(relay_frame_phase(relay_origin, frame, old_repair) == RELAY_REPAIR_START + 1U);

    /* Missing announced future start must not fall back to a guessed grid. */
    relay_timeline_active.valid &= (uint8_t)~(1U << ((frame + 1U) % RELAY_TIMELINE_FRAMES));
    assert(!relay_control_target(start, frame, true, &output_frame, &preamble));
    assert(!relay_control_target(current_preamble + 1U, 0, false, &output_frame, &preamble));
}

static unsigned fired;
static void probe_callback(int32_t channel, uint64_t tick, void *context)
{
    assert(channel == 1 && tick == 393 && context);
    ++fired;
}

static void compare_scheduling(void)
{
    reset_device();
    struct relay_control_work work = {0};
    assert(!relay_control_compare_ready(103, 101));
    assert(relay_control_compare_ready(104, 101));
    assert(!relay_control_compare_ready(101, 101));
    assert(!relay_control_compare_ready(100, 101));
    assert(relay_control_compare_ready(UINT64_C(0x100000006), UINT64_C(0x100000000)));

    clock_ticks = 100;
    assert(relay_control_schedule(0, 1, 109, probe_callback, &work) == -ETIME);
    assert(!exact_calls && aborts == 2 && relay_control_diag.preparation_late == 1);
    assert(!compares[0].armed && !compares[1].armed);

    clock_ticks = 100; exact_calls = aborts = 0; advance_after_launch = 2;
    assert(relay_control_schedule(0, 1, 110, probe_callback, &work) == -ETIME);
    assert(exact_calls == 1 && aborts == 2 && relay_control_diag.preparation_late == 2);
    assert(!compares[0].armed && !compares[1].armed && !compares[1].context);

    clock_ticks = 100; exact_calls = aborts = 0; advance_after_launch = 0; fail_on_exact = 1;
    injected_error = -EIO;
    assert(relay_control_schedule(0, 1, 120, probe_callback, &work) == -EIO);
    assert(exact_calls == 1 && aborts == 2 && relay_control_diag.launch_failed == 1);
    assert(relay_control_diag.launch_err == -EIO && relay_control_diag.callback_failed == 0);
    assert(!compares[0].armed && !compares[1].armed && !compares[1].context);

    clock_ticks = 100; exact_calls = aborts = 0; fail_on_exact = 2; injected_error = -EBUSY;
    assert(relay_control_schedule(0, 1, 120, probe_callback, &work) == -EBUSY);
    assert(exact_calls == 2 && aborts == 2 && relay_control_diag.callback_failed == 1);
    assert(relay_control_diag.callback_err == -EBUSY && relay_control_diag.launch_failed == 1);
    assert(!compares[0].armed && !compares[1].armed && !compares[1].context);
    assert(relay_control_diag.preparation_late == 2);

    clock_ticks = 200; exact_calls = aborts = 0; fail_on_exact = 0;
    assert(relay_control_schedule(0, 1, 400, probe_callback, &work) == 0);
    assert(exact_calls == 2 && !aborts && work.freeze_rtc == 395 && work.launch_rtc == 400);
    assert(compares[0].armed && compares[0].tick == 400);
    assert(compares[1].armed && compares[1].tick == 393 && compares[1].context == &work);
    compares[1].callback(1, compares[1].tick, compares[1].context);
    assert(fired == 1);
    z_nrf_rtc_timer_abort(0); z_nrf_rtc_timer_abort(1);
}

enum thread_fault { LATE, LAUNCH, CALLBACK, PREPARE, BEGIN, ARM, WAIT, FREEZE, SESSION, GENERATION };
static void thread_case(enum thread_fault fault)
{
    reset_device();
    const uint32_t frame = 14, start = 10000;
    setup_geometry(frame, start, 1, 1);
    relay_coord_init(&relay_coord);
    relay_coord.enabled = true;
    relay_coord.cache_frames = 5;
    relay_coord_note_caps(&relay_coord, 0, RELAY_INFO_CAPS);
    tdma_active_mask = 1;
    esb_initialized = true;
    relay_profile = 1; relay_epoch = 7; relay_session = 123; relay_layout_generation = 9;
    original_frame = frame; original_start = start;
    check_future_preparation = true;
    clock_ticks = fault == LATE ? start : start + RELAY_CONTROL_PREAMBLE + 1U;
    switch (fault) {
    case LATE: inject_late_prepare = true; break;
    case LAUNCH: fail_on_exact = 1; injected_error = -EIO; break;
    case CALLBACK: fail_on_exact = 2; injected_error = -EBUSY; break;
    case PREPARE: fail_prepare = true; break;
    case BEGIN: fail_begin = -EIO; break;
    case ARM: fail_arm = -EACCES; break;
    case WAIT: fail_wait = -ETIMEDOUT; break;
    case FREEZE: fail_freeze = true; break;
    case SESSION: fail_on_exact = 1; change_after_failure = 1; break;
    case GENERATION: fail_on_exact = 1; change_after_failure = 2; break;
    }
    if (!setjmp(stop_thread)) relay_control_thread();
    assert(!irq_depth && !compares[0].armed && !compares[1].armed);
    assert(!check_future_preparation && prepare_count == 2);
    assert(prepared_frames[0] == frame + (fault == LATE ? 0U : 1U));
    assert(prepared_frames[1] == frame + ((fault == LATE || fault == SESSION || fault == GENERATION) ? 1U : 2U));
    assert(relay_control_sent == 1 && relay_control_failed == 1);
    assert(relay_control_diag.preparation_late == (unsigned)(fault == LATE));
    assert(relay_control_diag.prepare_failed == (unsigned)(fault == PREPARE));
    assert(relay_control_diag.launch_failed == (unsigned)(fault == LAUNCH || fault == SESSION || fault == GENERATION));
    assert(relay_control_diag.callback_failed == (unsigned)(fault == CALLBACK));
    assert(relay_control_diag.begin_failed == (unsigned)(fault == BEGIN));
    assert(relay_control_diag.arm_failed == (unsigned)(fault == ARM));
    assert(relay_control_diag.wait_failed == (unsigned)(fault == WAIT));
    assert(relay_control_diag.freeze_timeout == (unsigned)(fault == FREEZE));
    assert(!relay_control_diag.stale_cancel);
    if (fault == LAUNCH || fault == SESSION || fault == GENERATION) assert(relay_control_diag.launch_err == -EIO);
    if (fault == CALLBACK) assert(relay_control_diag.callback_err == -EBUSY);
    if (fault == BEGIN) assert(relay_control_diag.begin_err == -EIO);
    if (fault == ARM) {
        assert(relay_control_diag.arm_err == -EACCES && relay_control_diag.failed_arm_seen);
        assert(relay_control_diag.failed_arm_frame == frame + 1U && !relay_control_diag.failed_arm_repeat);
        assert(relay_control_diag.failed_arm_entry_lag == 0);
        assert(relay_control_diag.failed_arm_before == 5 && relay_control_diag.failed_arm_after == 5);
        assert(relay_control_diag.failed_arm_err == -EACCES);
    }
    if (fault == WAIT) assert(relay_control_diag.wait_err == -ETIMEDOUT);
}

static void guarded_arm(unsigned mode, unsigned lag, unsigned work_ticks)
{
    reset_device();
    const uint32_t frame = 14, start = 0x00fffff0U;
    setup_geometry(frame, start, 1, mode ? 2 : 1);
    assert(relay_geometry_make_control(&geometry, 1, mode ? 2 : 1, 16, 120, mode));
    relay_applied_settings.control_repeat_mode = mode;
    relay_frame_announce(frame, start);
    relay_coord_init(&relay_coord);
    relay_coord.enabled = true;
    relay_coord.cache_frames = 5;
    relay_coord_note_caps(&relay_coord, 0, RELAY_INFO_CAPS);
    tdma_active_mask = 1;
    esb_initialized = true;
    relay_profile = 1; relay_epoch = 7; relay_session = 123; relay_layout_generation = 9;
    clock_ticks = start;
    freeze_lag = lag;
    arm_work = work_ticks;
    if (!setjmp(stop_thread)) relay_control_thread();
    assert(relay_control_sent == 1 && relay_control_failed == 0);
    assert(relay_control_repeat_sent == (mode != 0U) && !relay_control_repeat_failed);
    assert(relay_control_entry_max == lag && !relay_control_diag.failed_arm_seen);
    assert(!irq_depth && !compares[0].armed && !compares[1].armed);
}

static void failed_repeat_snapshot(void)
{
    reset_device();
    struct relay_control c = {.frame = UINT32_MAX - 1U};
    struct relay_control_work w = {.control = &c, .generation = 9, .start_event = 1, .repeat = true};
    relay_layout_generation = 9;
    const uint64_t launch = 0x01000004U;
    clock_ticks = launch - 30U;
    freeze_lag = 2;
    arm_work = 5;
    assert(!relay_control_schedule(0, 1, launch, relay_repeat_callback, &w));
    assert(w.freeze_rtc == launch - 5U);
    k_sem_take(&relay_freeze_done, K_FOREVER);
    assert(w.result == -ETIME && relay_control_late == 1);
    assert(relay_control_diag.failed_arm_seen && relay_control_diag.failed_arm_repeat);
    assert(relay_control_diag.failed_arm_frame == c.frame && relay_control_diag.failed_arm_entry_lag == 2);
    assert(relay_control_diag.failed_arm_before == 3 && relay_control_diag.failed_arm_after == -2);
    assert(relay_control_diag.failed_arm_err == -ETIME);
    relay_control_cancel(0, 1);
    assert(!compares[0].armed && !compares[1].armed);
    clock_ticks = launch + 10U;
    arm_work = 1;
    assert(!relay_control_schedule(0, 1, launch + 40U, relay_repeat_callback, &w));
    k_sem_take(&relay_freeze_done, K_FOREVER);
    assert(!w.result);
    assert(!esb_prx_tx_wait(100));
    atomic_set(&radio_operation_busy, 0);
    assert(relay_control_diag.failed_arm_frame == c.frame && relay_control_diag.failed_arm_before == 3);
    assert(relay_control_diag.failed_arm_after == -2 && relay_control_diag.failed_arm_err == -ETIME);
}

int main(void)
{
    /* 120-Hz rational padding has both 273 and 274 tick frames. */
    assert(relay_runtime_period(0, 1, 1, 16, 120, 0) == 273);
    assert(relay_runtime_period(14, 2, 1, 16, 120, 0) == 274);
    assert(relay_runtime_period(14, 16, 3, 16, 120, 0) == 324);
    for (unsigned i = 0; i < 3; ++i) {
        uint8_t direct = i == 0 ? 1 : i == 1 ? 2 : 16;
        uint8_t repairs = i == 2 ? 3 : 1;
        selection(0, 10000, direct, repairs);
        selection(14, 10000, direct, repairs);
        selection(UINT32_MAX, UINT32_MAX - 100U, direct, repairs);
    }
    compare_scheduling();
    for (unsigned mode = 0; mode <= 2; ++mode)
        for (unsigned lag = 0; lag <= 2; ++lag)
            for (unsigned work = 0; work <= 2; ++work) guarded_arm(mode, lag, work);
    failed_repeat_snapshot();
    for (enum thread_fault fault = LATE; fault <= GENERATION; ++fault) thread_case(fault);
    puts("Control target, live repair phase, late setup, compare cleanup and future retry passed");
}
'''

with tempfile.TemporaryDirectory(prefix='receiver-control-schedule-') as directory:
    temporary = Path(directory)
    generated = temporary / 'test.c'
    executable = temporary / 'test'
    generated.write_text(prefix + '\n' + production + '\n' + cases)
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unknown-attributes',
        '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
        '-fno-pie', '-no-pie', '-DCONFIG_RELAY_RUNTIME_LAYOUT=1',
        '-I' + str(ROOT / 'src/connection'),
        str(generated), str(ROOT / 'src/connection/relay_coord.c'), '-o', str(executable),
    ], check=True, timeout=60)
    subprocess.run([str(executable)], check=True, timeout=30)
