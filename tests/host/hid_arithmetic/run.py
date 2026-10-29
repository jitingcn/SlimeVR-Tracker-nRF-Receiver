#!/usr/bin/env python3
"""Exercise production HID arithmetic, byte packets, and roster snapshots with UBSan."""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=ROOT)
parser.add_argument('--case', choices=('rssi', 'uptime', 'roster', 'quaternion', 'all'), default='all')
args = parser.parse_args()
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness'))
from c_extract import extract_block

def function(text, name):
    return extract_block(text, r'^[\w \t*]+\b' + re.escape(name) +
                         r'\s*\([^;{}]*\)\s*\{') + '\n'

source = (args.source_root / 'src/hid.c').read_text()
declarations = source[source.index('struct rssi_ema_state {'):source.index('LOG_MODULE_REGISTER(hid_event')]
def constant(name):
    return re.search(r'^#define\s+' + name + r'\s+\d+', source, re.M).group()

prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#define MAX_TRACKERS 16
#define RCV_HID_TYPE_CMD_ACK 251
#define RCV_HID_TYPE_CMD 254
#define RCV_HID_TYPE_DEVICE_ADDR 255
#define ATOMIC_INIT(x) (x)
typedef int32_t atomic_t;
typedef int32_t atomic_val_t;
static int32_t atomic_get(const atomic_t *p) { return *p; }
static int32_t atomic_set(atomic_t *p, int32_t v) { int32_t old=*p; *p=v; return old; }
static int32_t atomic_add(atomic_t *p, int32_t v) { int32_t old=*p; *p+=v; return old; }
static bool atomic_cas(atomic_t *p, int32_t old, int32_t v) {
    if (*p != old) return false;
    *p=v; return true;
}
static uint64_t now;
static uint32_t k_uptime_get_32(void) { return (uint32_t)now; }
static int64_t k_uptime_get(void) { return (int64_t)now; }
static uint16_t sent_device_addr;
static uint8_t stored_trackers;
static uint8_t stored_tracker_addr[MAX_TRACKERS][6];
struct tracker_report { uint8_t data[16]; };
static bool delete_on_lock;
static unsigned int lock_depth;
static unsigned int irq_lock(void) {
    /* A roster deletion wins scheduling just before the snapshot lock. */
    if (delete_on_lock) { stored_trackers--; delete_on_lock=false; }
    return lock_depth++;
}
static void irq_unlock(unsigned int key) { assert(lock_depth == key+1); lock_depth=key; }
static void *checked_memcpy(void *dst, const void *src, size_t n) {
    if ((uintptr_t)src >= (uintptr_t)stored_tracker_addr &&
        (uintptr_t)src < (uintptr_t)(stored_tracker_addr+MAX_TRACKERS)) assert(lock_depth == 1);
    return memcpy(dst, src, n);
}
#define memcpy checked_memcpy
static uint8_t enqueued_packet[16];
static uint32_t enqueued_generation;
static unsigned int enqueued_count;
static bool hid_enqueue_packet(const uint8_t data[16], uint32_t generation) {
    memcpy(enqueued_packet, data, sizeof(enqueued_packet));
    enqueued_generation=generation;
    enqueued_count++;
    return true;
}
'''
functions = '\n'.join(function(source, name) for name in (
    'packet_device_addr', 'hid_fill_padding', 'hid_stats_record_reports',
    'hid_stats_update_idle_if_needed', 'hid_stats_snapshot', 'rssi_smooth_update',
    'hid_reset_rssi_smooth', 'hid_type_byte1_is_tracker_id',
    'hid_type_has_rssi_slot', 'hid_write_packet_n'))
# The historical address formatter returned void. Preserve its exact body and
# callers; only the test's direct boolean assertions need a signature adapter.
if re.search(r'^static void packet_device_addr\s*\(', source, re.M):
    functions += r'''
static bool legacy_packet_device_addr_result(uint8_t *report, uint16_t id) {
    packet_device_addr(report, id);
    return true;
}
#define packet_device_addr legacy_packet_device_addr_result
'''
checks = r'''
#undef memcpy
static int floor256(int x) { return x >= 0 ? x/256 : -((-x+255)/256); }
static void assert_rssi_report(uint8_t tracker_id, int8_t rssi, uint8_t expected) {
    uint8_t packet[16] = {0};
    packet[1]=tracker_id;
    packet[2]=0xA5;
    packet[15]=0x55;
    unsigned int before=enqueued_count;
    hid_write_packet_n(packet, (uint8_t)rssi);
    assert(enqueued_count==before+1 && enqueued_generation==0);
    assert(enqueued_packet[0]==0 && enqueued_packet[1]==tracker_id);
    assert(enqueued_packet[2]==0xA5 && enqueued_packet[15]==expected);
}
static void test_rssi(void) {
    for (int first=INT8_MIN; first<=INT8_MAX; first++) {
        hid_reset_rssi_smooth(0);
        assert(rssi_smooth_update(0, first) == (uint8_t)first);
        int q8=first*256;
        for (int sample=INT8_MIN; sample<=INT8_MAX; sample++) {
            q8=floor256(RSSI_EMA_ALPHA*sample*256 + (256-RSSI_EMA_ALPHA)*q8);
            assert(rssi_smooth_update(0, sample) == (uint8_t)floor256(q8));
            assert(rssi_states[0].ema_value == q8);
        }
    }
    assert(rssi_smooth_update(MAX_TRACKERS, -99) == (uint8_t)-99);
    hid_reset_rssi_smooth(0);
    hid_reset_rssi_smooth(15);
    assert_rssi_report(0, -80, 176);
    assert_rssi_report(15, -40, 216);
    assert_rssi_report(0, -60, 179);
    assert_rssi_report(15, -60, 212);
    hid_reset_rssi_smooth(0);
    assert_rssi_report(0, -100, 156);
    assert_rssi_report(15, -60, 208);
    hid_reset_rssi_smooth(MAX_TRACKERS);
    assert_rssi_report(MAX_TRACKERS, -99, 157);
    assert_rssi_report(0, -60, 163);
    assert_rssi_report(15, -60, 206);
    hid_reset_rssi_smooth(15);
    assert_rssi_report(15, -30, 226);
    assert_rssi_report(0, -60, 170);
}
static void test_quaternion(void) {
    _Alignas(uint16_t) uint8_t storage[17];
    volatile unsigned offsets[] = {0, 1};
    for (unsigned alignment=0; alignment<2; alignment++) {
        uint8_t *packet=storage+offsets[alignment];
        for (unsigned type=1; type<=4; type+=3) {
            memset(packet, 0xA5, 16);
            packet[0]=(uint8_t)type;
            memset(packet+2, 0, 8);
            unsigned before=enqueued_count;
            hid_write_packet_n(packet, 200);
            assert(enqueued_count==before);
            for (unsigned byte=2; byte<=9; byte++) {
                packet[byte]=1;
                hid_write_packet_n(packet, 200);
                assert(enqueued_count==++before && enqueued_generation==0);
                assert(!memcmp(enqueued_packet, packet, 16));
                packet[byte]=0;
            }
        }
    }
}
static void test_window(uint64_t start) {
    memset(&hid_stats, 0, sizeof(hid_stats));
    now=start;
    hid_stats_record_reports(0);
    hid_stats_update_idle_if_needed(k_uptime_get_32());
    assert(hid_stats_snapshot(NULL)==0);
    hid_stats_record_reports(3);
    now=start+999;
    hid_stats_record_reports(2);
    hid_stats_update_idle_if_needed(k_uptime_get_32());
    assert(hid_stats_snapshot(NULL)==0);
    now=start+1000;
    hid_stats_record_reports(4);
    bool active=false;
    assert(hid_stats_snapshot(&active)==9 && active);
    assert(hid_stats_snapshot(&active)==9 && !active);
    now=start+1999;
    hid_stats_update_idle_if_needed(k_uptime_get_32());
    assert(hid_stats_snapshot(NULL)==9);
    now=start+2000;
    hid_stats_update_idle_if_needed(k_uptime_get_32());
    assert(hid_stats_snapshot(NULL)==0);
    now=start+2001;
    hid_stats_record_reports(7);
    now=start+3000;
    hid_stats_update_idle_if_needed(k_uptime_get_32());
    assert(hid_stats_snapshot(NULL)==7);
    now=start+4000;
    hid_stats_update_idle_if_needed(k_uptime_get_32());
    assert(hid_stats_snapshot(NULL)==0);
}
static void test_uptime(void) {
    const uint64_t starts[]={0, 100, (uint64_t)INT32_MAX-500,
        (uint64_t)INT32_MAX+1, (uint64_t)UINT32_MAX-500,
        (uint64_t)UINT32_MAX-999, (uint64_t)UINT32_MAX+1,
        2*((uint64_t)UINT32_MAX+1)};
    for (unsigned i=0; i<sizeof(starts)/sizeof(starts[0]); i++) test_window(starts[i]);
}
static void assert_idle(const struct tracker_report *out) {
    assert(out->data[0]==0xF8);
    for (unsigned i=1; i<sizeof(out->data); i++) assert(out->data[i]==0);
}
static void test_roster(void) {
    struct tracker_report out;
    memset(stored_tracker_addr, 0xA5, sizeof(stored_tracker_addr));
    stored_trackers=2;
    sent_device_addr=1;
    hid_fill_padding(&out, 2);
    assert(out.data[0]==255 && out.data[1]==1);
    for (unsigned i=2; i<8; i++) assert(out.data[i]==0xA5);
    for (unsigned i=8; i<16; i++) assert(out.data[i]==0);
    sent_device_addr=1;
    delete_on_lock=true;
    hid_fill_padding(&out, 2);
    assert_idle(&out);
    assert(lock_depth==0 && stored_trackers==1);
    hid_fill_padding(&out, 1);
    assert(out.data[0]==255 && out.data[1]==0);
    delete_on_lock=true;
    hid_fill_padding(&out, 1);
    assert_idle(&out);
    stored_trackers=MAX_TRACKERS;
    assert(!packet_device_addr(out.data, MAX_TRACKERS));
    for (unsigned count=17; count<=255; count+=238) {
        stored_trackers=count;
        memset(&out, 0xCC, sizeof(out));
        hid_fill_padding(&out, count);
        assert_idle(&out);
        assert(!packet_device_addr(out.data, 0));
    }
}
int main(int argc, char **argv) {
    assert(argc==2);
    if (!strcmp(argv[1], "all") || !strcmp(argv[1], "rssi")) test_rssi();
    if (!strcmp(argv[1], "all") || !strcmp(argv[1], "uptime")) test_uptime();
    if (!strcmp(argv[1], "all") || !strcmp(argv[1], "roster")) test_roster();
    if (!strcmp(argv[1], "all") || !strcmp(argv[1], "quaternion")) test_quaternion();
    puts("HID arithmetic/snapshot behavior passed");
}
'''
with tempfile.TemporaryDirectory(prefix='receiver-hid-arithmetic-') as directory:
    path = Path(directory)
    fixture = path / 'fixture.c'
    fixture.write_text(prefix + '\n' + constant('RSSI_EMA_ALPHA') + '\n' +
                       constant('HID_TPS_UPDATE_INTERVAL_MS') + '\n' +
                       declarations + functions + checks)
    executable = path / 'fixture'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=c11', '-O2', '-Wall', '-Wextra', '-Wno-unused-function',
        '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
        str(fixture), '-o', str(executable)], check=True)
    subprocess.run([str(executable), args.case], check=True)
