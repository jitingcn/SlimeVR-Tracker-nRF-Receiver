#!/usr/bin/env python3
"""Run actual receiver event code; --wire-input emits actual client parser JSONL.

Replay input contains one hexadecimal ESB packet per line. @1234 advances the
absolute monotonic clock to 1234ms and processes timers. Blank/comment lines are
ignored. Without @ directives packet lines advance the clock 100ms; after the
first @ directive the input owns the clock until the next directive.
An exclamation-mark line starts a new subscription and requests fresh snapshots.
"""
import argparse
from collections import defaultdict, deque
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
import types

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness'))
from c_extract import extract_block

def function(text, name):
    return extract_block(text, r'^[\w \t*]+\b' + re.escape(name) +
                         r'\s*\([^;{}]*\)\s*\{') + '\n'

CASES = ('golden', 'decoder', 'timeout', 'storage', 'nonce', 'sequence',
         'long-heartbeat', 'subscriptions', 'rest', 'independent', 'pairing',
         'rest-long-sequence', 'rest-delivery-retry', 'json-rest', 'composite',
         'power-timeout', 'power-cancel', 'power-reanchor', 'power-expiry',
         'power-activity', 'power-new-operation', 'power-boot', 'power-clear',
         'power-reset', 'json-power', 'power-long-sequence', 'power-long-reanchor')

KERNEL = r'''
#ifndef HOST_KERNEL_H
#define HOST_KERNEL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
extern uint32_t host_now;
static inline uint32_t k_uptime_get_32(void) { return host_now; }
static inline int64_t k_uptime_get(void) { return host_now; }
struct k_spinlock { int unused; };
typedef unsigned int k_spinlock_key_t;
static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *s) { (void)s; return 0; }
static inline void k_spin_unlock(struct k_spinlock *s, k_spinlock_key_t key) { (void)s; (void)key; }
#define K_NO_WAIT 0
#define K_MSEC(ms) (ms)
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define BIT(n) (1U<<(n))
#define ARG_UNUSED(a) ((void)(a))
#define BUILD_ASSERT(c,...) _Static_assert(c, #c)
struct k_msgq { unsigned char *buffer; size_t item_size, capacity, read, count; };
#define __noinit
#define __aligned(n)
#define STRUCT_SECTION_ITERABLE(type, name) struct type name
#define Z_MSGQ_INITIALIZER(name, buf, size, num) \
    { .buffer=(unsigned char *)(buf), .item_size=(size), .capacity=(num) }
static inline int k_msgq_put(struct k_msgq *q,const void *item,int timeout) {
    (void)timeout;
    if(q->count==q->capacity) return -ENOMSG;
    memcpy(q->buffer+((q->read+q->count)%q->capacity)*q->item_size,item,q->item_size);
    q->count++; return 0;
}
static inline int k_msgq_get(struct k_msgq *q,void *item,int timeout) {
    (void)timeout;
    if(!q->count) return -ENOMSG;
    memcpy(item,q->buffer+q->read*q->item_size,q->item_size);
    q->read=(q->read+1)%q->capacity; q->count--; return 0;
}
static inline void k_msgq_purge(struct k_msgq *q) { q->read=q->count=0; }
#endif
'''
LOGGING = '''#include <stdarg.h>
#define LOG_MODULE_REGISTER(...)
static inline void host_log(const char *format, ...) { (void)format; }
#define LOG_INF(...) host_log(__VA_ARGS__)
#define LOG_WRN(...) host_log(__VA_ARGS__)
#define LOG_ERR(...) host_log(__VA_ARGS__)
#define LOG_DBG(...) host_log(__VA_ARGS__)
'''
def load_client(root):
    # Only the external USB transport dependency is stubbed. Parser and pump
    # are imported intact, and the mixed-report test supplies device.read().
    sys.modules.setdefault('hid', types.ModuleType('hid'))
    spec = importlib.util.spec_from_file_location('event_test_hid_cmd', root / 'scripts/hid_cmd.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def parser_cases(client, record):
    parsed = client.decode_tracker_event(record, 1234)
    assert parsed == {
        'source': 'tracker', 'snapshot': False, 'origin': 'user', 'tracker_id': 2,
        'boot_id': 0x12345678, 'operation_id': 0x2345, 'event_seq': 0x3456,
        'kind': 'IMU_ZRO', 'event': 'BEGIN', 'phase': 'COLLECT',
        'outcome': 'NONE', 'detail': 0, 'received_monotonic_ms': 1234,
    }
    ack = bytes((251, 7, 224, 0)) + bytes(range(12))
    pose = bytes((1, 2)) + bytes(14)
    report = record + ack + pose + record
    instance = client.HidCmdClient.__new__(client.HidCmdClient)
    class Device:
        def read(self, size, timeout):
            assert size == 64
            return report
    instance.dev = Device()
    instance.events = deque()
    instance.acks = defaultdict(deque)
    assert instance.pump(0)
    assert len(instance.events) == 2
    assert instance.acks[(7,224)].popleft() == (0, bytes(range(12)))
    assert [e['event_seq'] for e in instance.events] == [0x3456,0x3456]
    for length in (0,1,15,17):
        assert client.decode_tracker_event(record[:length] if length<16 else record+b'\0') is None
    assert client.decode_tracker_event(ack) is None
    def decoded_event(event, outcome, operation=42, source=225):
        wire=bytearray(record)
        wire[2]=source
        wire[3]=event | (outcome<<4)
        wire[11:13]=operation.to_bytes(2,'little')
        return client.decode_tracker_event(bytes(wire),1234)
    watch=client.CalibrationWatch(2,1,1000)
    assert watch.observe(decoded_event(4,1,41)) is None
    assert watch.observe(decoded_event(1,0)) is None
    assert watch.observe(decoded_event(4,5,source=226)) is None
    assert watch.observe(decoded_event(4,1))==0
    rejected=client.CalibrationWatch(2,1,1000)
    assert rejected.observe(decoded_event(5,2))==1
    assert rejected.observe(decoded_event(4,1,41))==1
    print('PASS python-golden-and-mixed-report')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path,
                        default=Path(os.environ.get('SOURCE_ROOT', Path(__file__).resolve().parents[3])))
    parser.add_argument('--case', action='append', choices=CASES)
    parser.add_argument('--wire-input', type=Path)
    parser.add_argument('--diagnostics', choices=['off', 'on'], help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.diagnostics is None:
        outputs = []
        for diagnostics in ('off', 'on'):
            completed = subprocess.run(
                [sys.executable, *sys.argv, '--diagnostics', diagnostics],
                text=True, capture_output=True)
            if completed.returncode:
                sys.stderr.write(completed.stderr)
                raise subprocess.CalledProcessError(completed.returncode, completed.args)
            outputs.append(completed.stdout)
        assert outputs[0] == outputs[1], 'diagnostics changed protocol/event output'
        sys.stdout.write(outputs[0])
        return
    root = args.source_root.resolve()
    client = load_client(root)
    with tempfile.TemporaryDirectory(prefix='receiver-events-') as directory:
        path = Path(directory)
        (path/'zephyr/logging').mkdir(parents=True)
        (path/'zephyr/sys').mkdir()
        (path/'zephyr/kernel.h').write_text(KERNEL)
        (path/'zephyr/logging/log.h').write_text(LOGGING)
        (path/'zephyr/sys/util.h').write_text('#include <zephyr/kernel.h>\n')
        esb = (root/'src/connection/esb.c').read_text()
        extract_spec = importlib.util.spec_from_file_location(
            'tdma_loss_extract', Path(__file__).resolve().parent.parent / 'tdma_loss/run.py')
        extract = importlib.util.module_from_spec(extract_spec)
        extract_spec.loader.exec_module(extract)
        layout_constants = '\n'.join(re.findall(
            r'^#define TDMA_(?:BASE_SLOT_TICKS|SLOT_TICKS_MAX_LEVEL0|'
            r'RECONFIG_MIN_MS|LOSS_RECONFIG_MIN_MS|CAP_LEVEL_MAX)\b[^\n]*', esb, re.M))
        layout_names = '''tdma_cap_ladder tdma_config_packed tdma_config_epoch tdma_dynamic_active_count
            tdma_dynamic_slot_ticks tdma_active_mask tdma_cap_level tdma_published_cap_level
            tdma_loss_trigger_streak_ticks tdma_loss_recover_streak_ticks
            tdma_loss_probe_streak_ticks tdma_loss_pool_received tdma_loss_pool_gaps
            tdma_loss_pool_ticks'''.split()
        declarations = '\n'.join([layout_constants, extract.membership_declarations(esb)] +
                                 [extract.declaration(esb, name) for name in layout_names])
        declarations += '\n' + extract.diagnostic_loss_declarations(esb)
        (path/'esb_membership_declarations.inc').write_text(declarations)
        membership = '\n'.join(function(esb, name) for name in (
            'tdma_loss_reset_windows', 'tdma_pack_config', 'tdma_shadow_observe',
            'tdma_shadow_forget_tracker', 'tdma_shadow_merge_locked',
            'tdma_slot_ticks_for', 'tdma_recalculate'))
        lengths = function(esb, 'composite_body_length')
        composite = function(esb, 'esb_receive_composite')
        clear = function(esb, 'esb_clear')
        (path/'esb_event_paths.inc').write_text(
            membership + '\n' + lengths + '''
static void tracked_tdma_shadow_observe(uint8_t tracker, enum tdma_shadow_evidence kind) {
    assert(tracker==2 && kind==TDMA_SHADOW_EVIDENCE_DATA);
    evidence_calls++;
    tdma_shadow_observe(tracker, kind);
}
#define tdma_shadow_observe tracked_tdma_shadow_observe
''' + composite + '\n#undef tdma_shadow_observe\n' + clear)
        binary = path/'test'
        command = shlex.split(os.environ.get('CC','cc')) + [
            '-std=c11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
            '-Wno-unused-parameter','-I'+str(path),'-I'+str(root/'src'),
            str(root/'src/tracker_events.c'), str(Path(__file__).with_name('fixture.c')),
            '-o',str(binary),
        ]
        if args.diagnostics == 'on':
            command.append('-DCONFIG_TDMA_DIAGNOSTICS=1')
        subprocess.run(command,check=True)
        if args.wire_input is not None:
            completed = subprocess.run([str(binary),'replay'],input=args.wire_input.read_text(),
                                       text=True,capture_output=True,check=True)
            for line in completed.stdout.splitlines():
                timestamp,hex_record=line.split()
                decoded = client.decode_tracker_event(bytes.fromhex(hex_record), int(timestamp))
                assert decoded is not None
                print(json.dumps(decoded,sort_keys=True))
            return
        if args.case is None:
            logging_spec = importlib.util.spec_from_file_location(
                'event_test_logging', Path(__file__).with_name('logging_cases.py'))
            logging_module = importlib.util.module_from_spec(logging_spec)
            logging_spec.loader.exec_module(logging_module)
            logging_module.logging_cases(client)
        for case in args.case or CASES:
            completed = subprocess.run([str(binary),case],text=True,capture_output=True)
            if completed.returncode:
                sys.stderr.write(completed.stderr)
                raise subprocess.CalledProcessError(completed.returncode, [str(binary),case])
            if case=='golden':
                parser_cases(client,bytes.fromhex(completed.stdout.strip()))
            if case=='json-rest':
                decoded=[client.decode_tracker_event(bytes.fromhex(line.split()[1]),int(line.split()[0]))
                         for line in completed.stdout.splitlines()]
                assert [e['source'] for e in decoded]==['receiver']*4
                assert [e['snapshot'] for e in decoded]==[True,False,True,True]
                assert [e['phase'] for e in decoded]==['REST','UNKNOWN','REST','REST']
                assert [e['outcome'] for e in decoded]==['NONE','UNKNOWN','NONE','NONE']
            if case=='json-power':
                decoded=[client.decode_tracker_event(bytes.fromhex(line.split()[1]),int(line.split()[0]))
                         for line in completed.stdout.splitlines()]
                assert [e['phase'] for e in decoded]==[
                    'WILL_WOM','WILL_SHUTDOWN','BOOT','WAKE','WILL_REBOOT',
                    'WOM_CANCELLED','WATCHDOG_RESET']
                assert [e['detail'] for e in decoded]==['WOM_NORMAL',0,0,0,0,'WOM_FORCED',0]
                assert all(e['source']=='tracker' and not e['snapshot'] and e['event']=='NOTICE'
                           and e['outcome']=='NONE' and e['origin'] is None for e in decoded)
            print('PASS '+case,flush=True)


if __name__ == '__main__':
    main()
