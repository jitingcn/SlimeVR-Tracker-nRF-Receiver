#!/usr/bin/env python3
"""Exercise test-all layout publication against a pending targeted command."""
import importlib.util
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
spec = importlib.util.spec_from_file_location("extract", HERE.parent / "tdma_loss/run.py")
extract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extract)
source = (ROOT / "src/connection/esb.c").read_text()
header = (ROOT / "src/connection/esb.h").read_text()


def define(text, name):
    match = re.search(r"^#define " + re.escape(name) + r"\b[^\n]*", text, re.MULTILINE)
    if match is None:
        raise ValueError(f"Missing definition: {name}")
    return match.group()


def instrument(name):
    body = extract.function(source, name)
    patterns = (
        r"(if\s*\(\s*!atomic_get\(&test_all_state_valid\)[^{]*\{[^}]*\})",
        r"(atomic_and\(&test_all_confirmed_mask,[^;]*;)",
        r"(tracker_slots\[i\]\.tracker_test_on_tps\s*=[^;]*;)",
        r"(atomic_set\(\s*&test_all_ready_after_ms\[i\],[^;]*;)",
    )
    if name == "test_all_update_effective":
        patterns = (r"(test_all_effective_tps\s*=[^;]*;)",) + patterns
    for pattern in patterns:
        body, count = re.subn(pattern, r"\1\n\thost_targeted_preempt();", body)
        if count != 1:
            raise ValueError(f"{name}: expected one publication site for {pattern}, found {count}")
    return body


constants = "\n".join(define(source, name) for name in (
    "TEST_ALL_RATE_QUANTUM_TPS", "TEST_ALL_JOIN_CONFIG_GRACE_MS", "TDMA_BASE_SLOT_TICKS",
))
constants += "\n" + "\n".join(define(header, name) for name in (
    "ESB_PONG_FLAG_TEST_MODE_ON", "ESB_PONG_FLAG_TEST_MODE_OFF",
))
structures = (extract.block(source, source.index("struct packet_stats {")) + ";\n" +
              extract.block(source, source.index("struct tracker_slot {")) + ";\n")
prelude = r'''
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MAX_TRACKERS 4
#define BIT(n) (1U << (n))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
typedef int32_t atomic_t;
typedef int32_t atomic_val_t;
static int atomic_get(const atomic_t *p) { return *p; }
static void atomic_set(atomic_t *p, int value) { *p = value; }
static int atomic_inc(atomic_t *p) { int old = *p; ++*p; return old; }
static void atomic_and(atomic_t *p, int mask) { *p &= mask; }
static unsigned irq_depth, attempts, deferred, deliveries, reclamp_logs;
static bool target_armed, target_pending, target_running;
static void run_targeted(void);
static unsigned int irq_lock(void) { return irq_depth++; }
static void irq_unlock(unsigned int key) {
    assert(irq_depth == key + 1);
    irq_depth = key;
    if (!irq_depth && target_pending && !target_running) {
        target_pending = false;
        run_targeted();
    }
}
static void host_targeted_preempt(void) {
    if (!target_armed) return;
    ++attempts;
    if (irq_depth || target_running) {
        ++deferred;
        target_pending = true;
    } else {
        run_targeted();
    }
}
static void host_log(const char *format, ...) {
    assert(!irq_depth);
    if (strstr(format, "Sticky TEST_MODE_ON reclamp:")) {
        va_list args;
        va_start(args, format);
        assert(va_arg(args, int) == 500);
        assert(va_arg(args, int) == 250);
        assert(va_arg(args, int) == 2);
        assert(va_arg(args, int) == 250);
        va_end(args);
        ++reclamp_logs;
    }
}
#define LOG_INF(...) host_log(__VA_ARGS__)
'''
state = r'''
static struct tracker_slot tracker_slots[MAX_TRACKERS];
static atomic_t test_all_state_valid, test_all_enabled, test_all_broadcasting;
static atomic_t test_all_confirmed_mask, test_all_generation;
static atomic_t test_all_ready_after_ms[MAX_TRACKERS];
static uint16_t test_all_requested_tps, test_all_effective_tps;
static uint8_t tdma_dynamic_slot_ticks;
static uint32_t tdma_active_mask;
'''
helpers = "\n".join(extract.function(source, name) for name in (
    "test_all_capacity_tps", "test_all_requested_value", "test_all_wire_tps",
    "test_all_invalidate", "esb_send_remote_command_test_on",
))
helpers += "\n" + "\n".join(instrument(name) for name in (
    "test_all_note_layout", "test_all_update_effective",
))
cases = r'''
static void run_targeted(void) {
    assert(!irq_depth && !target_running);
    target_armed = false;
    target_running = true;
    ++deliveries;
    assert(esb_send_remote_command_test_on(0, 0x2468) == BIT(0));
    target_running = false;
}
static void reset(void) {
    memset(tracker_slots, 0, sizeof(tracker_slots));
    memset(test_all_ready_after_ms, 0, sizeof(test_all_ready_after_ms));
    test_all_state_valid = test_all_enabled = 1;
    test_all_confirmed_mask = BIT(0) | BIT(1);
    test_all_broadcasting = test_all_generation = 0;
    test_all_requested_tps = test_all_effective_tps = 500;
    tdma_dynamic_slot_ticks = 64;
    tdma_active_mask = BIT(0) | BIT(1);
    irq_depth = attempts = deferred = deliveries = reclamp_logs = 0;
    target_armed = true;
    target_pending = target_running = false;
}
static void assert_target_won(unsigned expected_attempts) {
    assert(!irq_depth && !target_pending && !target_armed);
    assert(attempts == expected_attempts && deferred == expected_attempts && deliveries == 1);
    assert(!test_all_state_valid && !test_all_confirmed_mask && !test_all_broadcasting);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_TEST_MODE_ON);
    assert(tracker_slots[0].tracker_test_on_tps == 0x2468);
}
int main(void) {
    reset();
    test_all_note_layout(0, BIT(0) | BIT(1), 1000);
    assert_target_won(6);
    assert(tracker_slots[1].tracker_test_on_tps == 500);
    assert(test_all_ready_after_ms[0] == 2500 && test_all_ready_after_ms[1] == 2500);
    assert(!reclamp_logs);

    reset();
    test_all_update_effective(2, 1000, true);
    assert_target_won(7);
    assert(test_all_effective_tps == 250 && tracker_slots[1].tracker_test_on_tps == 250);
    assert(test_all_ready_after_ms[0] == 2500 && test_all_ready_after_ms[1] == 2500);
    assert(reclamp_logs == 1);

    reset();
    target_armed = false;
    test_all_state_valid = 0;
    test_all_note_layout(0, BIT(0), 1000);
    test_all_update_effective(2, 1000, true);
    assert(!irq_depth && !attempts && !reclamp_logs);
    test_all_state_valid = 1;
    test_all_note_layout(BIT(0), BIT(0), 1000);
    test_all_enabled = 0;
    test_all_update_effective(2, 1000, true);
    assert(!irq_depth && !attempts && !reclamp_logs);
    puts("PASS test-all layout publication excludes targeted command preemption");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="receiver-policy-publication-") as directory:
    path = Path(directory)
    generated = path / "test.c"
    generated.write_text(prelude + constants + "\n" + structures + state + helpers + cases)
    binary = path / "test"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
        str(generated), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
