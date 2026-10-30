#!/usr/bin/env python3
"""Compile production OTA/CDC/HID forget paths with deterministic host leaves."""
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
args = parser.parse_args()
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness'))
from c_extract import extract_block

def function(text, name):
    return extract_block(text, r'^[\w \t*]+\b' + re.escape(name) +
                         r'\s*\([^;{}]*\)\s*\{') + '\n'

common = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#define MAX_TRACKERS 16
#define BIT(n) (1U << (n))
#define LOG_INF(...)
typedef int32_t atomic_t;
typedef int32_t atomic_val_t;
static unsigned int depth;
static unsigned int irq_lock(void) { return depth++; }
static void irq_unlock(unsigned int key) { assert(depth == key+1); depth=key; }
static int32_t atomic_get(const atomic_t *p) { return *p; }
static int32_t atomic_set(atomic_t *p, int32_t v) { int32_t old=*p; *p=v; return old; }
'''
ota = (args.source_root / 'src/esb_ota.c').read_text()
ota_decls = ota[ota.index('struct ota_tx_entry {'):ota.index('static void fw_info_send_work_handler')]
ota_fixture = common + r'''
#define OTA_MAX_PARALLEL 4
#define OTA_TX_RING_SIZE 128
struct k_spinlock { unsigned int unused; };
typedef unsigned int k_spinlock_key_t;
static bool relay_locked;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock) {
    (void)lock;
    assert(!relay_locked);
    relay_locked=true;
    return irq_lock();
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key) {
    (void)lock;
    assert(relay_locked);
    relay_locked=false;
    irq_unlock(key);
}
static unsigned int releases;
static void unsuppress_all_trackers(void) { assert(depth==0 && !relay_locked); releases++; }
''' + ota_decls + '\n'.join(function(ota, name) for name in (
    'remove_target_locked', 'esb_ota_forget_tracker')) + r'''
int main(void) {
    ota_relay.active=true;
    ota_relay.num_targets=2;
    ota_relay.target_ids[0]=2;
    ota_relay.target_ids[1]=5;
    ota_relay.ring_head=29;
    ota_relay.tracker[2].participating=true;
    ota_relay.tracker[5].participating=true;
    ota_relay.tracker[2].status=3;
    ota_relay.tracker[2].pending_cmd_type=0xF2;
    ota_relay.tracker[2].pending_cmd_len=64;
    ota_relay.tracker[2].pending_cmd_send_count=10;
    ota_relay.tracker[2].next_seq=17;
    memset(ota_relay.tracker[2].pending_cmd_data, 0xCC, 64);
    ota_abort_valid[2]=true;
    ota_abort_expiry_ticks[2]=100;
    fw_info_pending.pending=true;
    fw_info_pending.tracker_id=2;
    esb_ota_forget_tracker(2);
    assert(!depth && releases==0 && ota_relay.active);
    assert(ota_relay.num_targets==1 && ota_relay.target_ids[0]==5);
    assert(ota_relay.ring_head==29 && ota_relay.tracker[5].participating);
    const unsigned char *slot=(const unsigned char *)&ota_relay.tracker[2];
    for (size_t i=0; i<sizeof(ota_relay.tracker[2]); i++) assert(slot[i]==0);
    assert(!ota_abort_valid[2] && ota_abort_expiry_ticks[2]==0 && !fw_info_pending.pending);
    fw_info_pending.pending=true;
    fw_info_pending.tracker_id=7;
    esb_ota_forget_tracker(2);
    assert(releases==0 && fw_info_pending.pending);
    esb_ota_forget_tracker(5);
    assert(!ota_relay.active && ota_relay.num_targets==0 && ota_relay.ring_head==0);
    assert(releases==1 && depth==0);
    ota_abort_valid[7]=true;
    ota_abort_expiry_ticks[7]=200;
    esb_ota_forget_tracker(7);
    assert(!ota_abort_valid[7] && !fw_info_pending.pending && releases==1);
    esb_ota_forget_tracker(255);
    assert(releases==1 && depth==0);
    puts("OTA slot forget behavior passed");
}
'''
collection_common = common + r'''
static bool dc_active, dc_batch_active;
static uint8_t dc_target_tracker_id;
static uint32_t dc_batch_mask;
'''
collection_checks = r'''
static void check_membership(void) {
    dc_active=true;
    dc_target_tracker_id=2;
    dc_batch_active=false;
    dc_batch_mask=0;
    data_collect_forget_tracker(5);
    assert(dc_active && dc_target_tracker_id==2);
    data_collect_forget_tracker(2);
    assert(!dc_active && dc_target_tracker_id==UINT8_MAX && !depth);
    dc_batch_active=true;
    dc_batch_mask=BIT(2)|BIT(5);
    data_collect_forget_tracker(2);
    assert(dc_batch_active && dc_batch_mask==BIT(5) && !depth);
    data_collect_forget_tracker(5);
    assert(!dc_batch_active && dc_batch_mask==0 && !depth);
    data_collect_forget_tracker(255);
    assert(!depth && dc_batch_mask==0);
}
'''
cdc = (args.source_root / 'src/data_collect.c').read_text()
cdc_fixture = collection_common + r'''
static volatile uint32_t dc_buf_head, dc_buf_tail;
''' + function(cdc, 'dc_discard_buffer') + function(cdc, 'data_collect_forget_tracker') + collection_checks + r'''
int main(void) {
    dc_buf_head=201; dc_buf_tail=19;
    check_membership();
    assert(dc_buf_head==201 && dc_buf_tail==201);
    dc_buf_head=4; dc_buf_tail=16000;
    data_collect_forget_tracker(0);
    assert(dc_buf_tail==4);
    puts("CDC slot forget behavior passed");
}
'''
hid = (args.source_root / 'src/data_collect_hid.c').read_text()
hid_fixture = collection_common + r'''
#define DC_FIFO_SIZE 64U
static atomic_t dc_fifo_read, dc_fifo_write;
static const uint8_t *dc_submitted_report;
''' + '\n'.join(function(hid, name) for name in (
    'dc_fifo_next', 'dc_reset_fifo_locked', 'data_collect_forget_tracker')) + collection_checks + r'''
int main(void) {
    dc_fifo_read=5; dc_fifo_write=20;
    check_membership();
    assert(dc_fifo_read==20 && dc_fifo_write==20);
    static const uint8_t inflight[64]={0x13};
    dc_submitted_report=inflight;
    dc_fifo_read=63; dc_fifo_write=7;
    data_collect_forget_tracker(2);
    assert(dc_fifo_read==63 && dc_fifo_write==0 && dc_submitted_report==inflight);
    dc_submitted_report=NULL;
    dc_fifo_read=0;
    assert(dc_fifo_read==dc_fifo_write);
    puts("HID slot forget behavior passed");
}
'''
with tempfile.TemporaryDirectory(prefix='receiver-slot-lifecycle-') as directory:
    path = Path(directory)
    for name, source in [('ota', ota_fixture), ('cdc', cdc_fixture), ('hid', hid_fixture)]:
        fixture = path / (name + '.c')
        fixture.write_text(source)
        executable = path / name
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=c11', '-O2', '-Wall', '-Wextra', '-Wno-unused-function',
            '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
            str(fixture), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
