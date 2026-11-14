#!/usr/bin/env python3
"""Compile current production receiver C bodies against deterministic host leaves.
Usage: python3 run.py [--source-root RECEIVER] [--case CASE]
SOURCE_ROOT may point at a saved pre-change source tree.
No source-text assertions: all cases exercise queue, scheduler, USB and UART behavior.
"""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

import argparse
import os
import importlib.util
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=Path(os.environ.get('SOURCE_ROOT', Path(__file__).resolve().parents[3])))
parser.add_argument('--case', action='append')
args = parser.parse_args()
src = args.source_root / 'src'

def function(text, name):
    # Mask comments/literals without moving offsets: signatures may have a
    # trailing comment, and braces in comments/strings are not C delimiters.
    non_code = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', re.DOTALL)
    code = non_code.sub(lambda match: re.sub(r'[^\n]', ' ', match.group()), text)
    match = re.search(r'^[\w \t*]+\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{', code, re.M)
    if match is None:
        raise ValueError(f"Missing production function: {name}")
    begin = code.index('{', match.start())
    depth = 0
    for token in re.finditer(r'[{}]', code[begin:]):
        depth += 1 if token.group() == '{' else -1
        if depth == 0:
            return text[match.start():begin + token.end()] + '\n'
    raise ValueError(f"Unclosed production function: {name}")

common = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#define __packed __attribute__((packed))
#define __aligned(n) __attribute__((aligned(n)))
#define ARG_UNUSED(x) (void)(x)
#define BUILD_ASSERT(c, m) _Static_assert(c, m)
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...)
#define LOG_WRN(...)
#define LOG_ERR(...)
#define BIT(n) (1U << (n))
#define MAX_TRACKERS 8
#define K_NO_WAIT 0
#define K_MSEC(x) (x)
#define K_SECONDS(x) ((x) * 1000)
#define MIN(a,b) ((a) < (b) ? (a) : (b))
typedef int32_t atomic_t;
typedef int32_t atomic_val_t;
#define ATOMIC_INIT(x) (x)
#define ATOMIC_DEFINE(n, bits) atomic_t n[1]
static void (*cas_hook)(void);
static bool atomic_cas(atomic_t *a, int32_t old, int32_t v) {
    if (*a!=old) return false;
    *a=v;
    if (cas_hook) { void (*fn)(void)=cas_hook; cas_hook=NULL; fn(); }
    return true;
}
static int32_t atomic_set(atomic_t *a, int32_t v) { int32_t old=*a; *a=v; return old; }
static int32_t atomic_add(atomic_t *a, int32_t v) { int32_t old=*a; *a+=v; return old; }
static int32_t atomic_get(const atomic_t *a) { return *a; }
static unsigned int irq_lock(void) { return 0; }
static void irq_unlock(unsigned int key) { (void)key; }
static bool atomic_test_and_set_bit(atomic_t *a, int bit) { bool old=(*a & BIT(bit))!=0; *a |= BIT(bit); return old; }
static bool atomic_test_and_clear_bit(atomic_t *a, int bit) { bool old=(*a & BIT(bit))!=0; *a &= ~BIT(bit); return old; }
static void atomic_clear_bit(atomic_t *a, int bit) { *a &= ~BIT(bit); }
struct device { int unused; } device;
struct k_work { void (*handler)(struct k_work *); int64_t due; bool queued; unsigned runs; };
#define k_work_delayable k_work
struct k_timer { void (*handler)(struct k_timer *); int64_t due, period; bool active; unsigned runs; };
#define K_WORK_DEFINE(name, fn) struct k_work name = {.handler=fn}
#define K_WORK_DELAYABLE_DEFINE(name, fn) K_WORK_DEFINE(name, fn)
#define K_TIMER_DEFINE(name, fn, stop) struct k_timer name = {.handler=fn}
static struct k_work *works[16];
static struct k_timer *timers[16];
static size_t nworks, ntimers;
static int64_t now;
uint32_t host_now;
static uint32_t k_uptime_get_32(void) { return (uint32_t)now; }
static int64_t k_uptime_get(void) { return now; }
static int64_t k_uptime_ticks(void) { return now * 32; }
static void k_work_init(struct k_work *w, void (*fn)(struct k_work *)) { w->handler=fn; }
#define k_work_init_delayable k_work_init
static int k_work_schedule(struct k_work *w, int delay) {
    bool known=false;
    for (size_t i=0; i<nworks; i++) if (works[i]==w) known=true;
    if (!known) { assert(nworks<16); works[nworks++]=w; }
    if (!w->queued) { w->queued=true; w->due=now+delay; }
    return 1;
}
static int k_work_reschedule(struct k_work *w, int delay) { w->queued=false; return k_work_schedule(w,delay); }
static int k_work_submit(struct k_work *w) { return k_work_schedule(w, 0); }
static void k_timer_start(struct k_timer *t, int delay, int period) {
    bool known=false;
    for (size_t i=0; i<ntimers; i++) if (timers[i]==t) known=true;
    if (!known) { assert(ntimers<16); timers[ntimers++]=t; }
    t->due=now+delay; t->period=period; t->active=true;
}
static void k_timer_stop(struct k_timer *t) { t->active=false; }
static void advance(int delta) {
    int64_t end=now+delta;
    for (unsigned step=0; ; step++) {
        assert(step<100000); /* Detect zero-delay retry/workqueue starvation. */
        int64_t due=INT64_MAX;
        for (size_t i=0; i<nworks; i++) if (works[i]->queued && works[i]->due<due) due=works[i]->due;
        for (size_t i=0; i<ntimers; i++) if (timers[i]->active && timers[i]->due<due) due=timers[i]->due;
        if (due>end) break;
        now=due;
        host_now=(uint32_t)now;
        for (size_t i=0; i<ntimers; i++) {
            struct k_timer *t=timers[i];
            if (t->active && t->due==now) { t->runs++; t->active=t->period!=0; t->due+=t->period; t->handler(t); }
        }
        for (size_t i=0; i<nworks; i++) {
            struct k_work *w=works[i];
            if (w->queued && w->due<=now) { w->queued=false; w->runs++; w->handler(w); }
        }
    }
    now=end;
    host_now=(uint32_t)now;
}
#define DT_NODELABEL(x) 0
#define DEVICE_DT_GET(x) (&device)
static bool device_is_ready(const struct device *dev) { return true; }
'''

hid_source = select_relay((src / 'hid.c').read_text())
hid_decls = hid_source[hid_source.index('static struct k_work'):hid_source.index('static const uint8_t hid_report_desc')]
hid_functions = '\n'.join(function(hid_source, name) for name in [
    'packet_device_addr', 'hid_stats_record_reports', 'hid_fill_padding', 'send_report',
    'int_in_ready_cb', 'input_report_done_cb', 'iface_ready_cb',
    'report_event_handler', 'hid_usb_state_changed', 'composite_pre_init',
    'hid_type_byte1_is_tracker_id', 'hid_type_has_rssi_slot',
    'hid_enqueue_packet', 'hid_write_tracker_event', 'hid_try_write_packet_n', 'hid_write_packet_n'])
hid = common + r'''
#define RCV_HID_TYPE_CMD_ACK 251
#define RCV_HID_TYPE_CMD 254
#include "tracker_events.h"
#include "connection/tracker_event_protocol.h"
#define RCV_HID_TYPE_DEVICE_ADDR 255
#define RCV_HID_CMD_LEN 16
static uint8_t stored_trackers;
static uint8_t stored_tracker_addr[MAX_TRACKERS][6];
static bool enabled=true, configured=true;
static bool receiver_usb_is_enabled(void) { return enabled; }
static bool receiver_usb_is_configured(void) { return configured; }
static int hid_device_register(const struct device *d, const void *desc, size_t n, const void *ops) { return 0; }
static void receiver_usb_set_state_callback(void (*cb)(bool)) { }
static void hid_async_cmd_ack(const uint8_t ack[16]) { }
static void rcv_cmd_set_async_ack(void (*cb)(const uint8_t *)) { }
static const uint8_t hid_report_desc[1];
static const int ops;
static uint16_t sent_device_addr;
static int64_t last_registration_sent;
static uint32_t dropped_reports, max_dropped_reports, total_dropped_reports;
static uint32_t tracker_drops[MAX_TRACKERS], total_tracker_drops[MAX_TRACKERS];
static int8_t rssi_smooth_update(uint8_t id, int8_t rssi) { return rssi; }
static uint8_t submitted[128][64];
static const uint8_t *inflight;
static unsigned submissions, attempts;
static int failures;
static int hid_device_submit_report(const struct device *d, size_t n, const uint8_t *p) {
    assert(n==64); attempts++;
    if (failures) { failures--; return failures ? -EAGAIN : -EACCES; }
    assert(inflight==NULL); assert(submissions<128);
    memcpy(submitted[submissions++], p, n); inflight=p; return 0;
}
''' + hid_decls + hid_functions + r'''
static void publish(unsigned id, uint8_t type) { uint8_t p[16]={0}; p[0]=type; p[1]=(uint8_t)id; p[2]=(uint8_t)(id+1); hid_write_packet_n(p, 0); }
static void done(void) { assert(inflight); const uint8_t *p=inflight; inflight=NULL; input_report_done_cb(hdev, p); }
static void setup(void) { assert(composite_pre_init()==0); iface_ready_cb(hdev, true); hid_usb_state_changed(true); advance(2); }
static void publish_behind_reserved_head(void) { publish(1,240); advance(2); assert(submissions==0); }
static uint32_t event_record(uint8_t record[16]) {
    uint8_t args[]={1,1,255,31}, result[12];
    assert(tracker_events_control(args,4,tracker_events_usb_generation(),result)==0);
    /* Capture the real generation by peeking the actual event producer. */
    struct tracker_event e={.nonce=1,.tracker_id=2,.operation_id=1,.event_seq=1,
        .kind=CAL_KIND_IMU_ZRO,.event=CAL_EVENT_BEGIN,.phase=CAL_PHASE_COLLECT};
    uint8_t wire[17]; assert(tracker_event_encode(wire,&e));
    assert(tracker_events_receive(wire,17,(uint32_t)now));
    tracker_events_process((uint32_t)now);
    struct tracker_report out;
    uint32_t generation;
    assert(hid_fifo_peek_batch(&out,&generation,1)==1);
    memcpy(record,out.data,16);
    return generation;
}
int main(int argc, char **argv) {
    assert(argc==2); setup();
    if (!strcmp(argv[1], "hid-idle")) {
        unsigned before=report_send.runs; advance(1000); assert(submissions==0); assert(report_send.runs==before);
    } else if (!strcmp(argv[1], "hid-producer-completion")) {
        k_timer_stop(&event_timer);
        for (unsigned i=0; i<9; i++) publish(i, 240);
        advance(2); assert(submissions==1);
        uint8_t held[64]; memcpy(held, inflight, 64); advance(20); assert(!memcmp(held, inflight, 64)); assert(submissions==1);
        done(); advance(2); assert(submissions==2); done(); advance(2); assert(submissions==3);
        for (unsigned i=0; i<9; i++) assert(submitted[i/4][(i%4)*16+1]==i);
        done(); advance(2); unsigned before=report_send.runs; advance(20); assert(report_send.runs==before); assert(hid_fifo_is_empty());
    } else if (!strcmp(argv[1], "hid-retry")) {
        k_timer_stop(&event_timer); failures=2; publish(3, 240); advance(1);
        assert(attempts==1 && submissions==0);
        advance(1); assert(attempts==2 && submissions==0);
        advance(1); assert(submissions==1 && submitted[0][1]==3);
        done(); advance(10); assert(submissions==1);
    } else if (!strcmp(argv[1], "hid-ready")) {
        k_timer_stop(&event_timer); configured=false; hid_usb_state_changed(false); publish(5, 240); advance(2); assert(submissions==0);
        iface_ready_cb(hdev, true); advance(2); assert(submissions==0);
        configured=true; hid_usb_state_changed(true); advance(2); assert(submissions==1); assert(submitted[0][1]==5);
        done(); configured=false; hid_usb_state_changed(false); publish(6, 240); advance(2);
        configured=true; hid_usb_state_changed(true); advance(2); assert(submissions==1);
        iface_ready_cb(hdev, true); advance(2); assert(submissions==2 && submitted[1][1]==6);
    } else if (!strcmp(argv[1], "hid-reserved-head")) {
        k_timer_stop(&event_timer);
        /* Preempt the actual head producer after its CAS, before seq publication. */
        cas_hook=publish_behind_reserved_head; publish(0,240); advance(2);
        assert(submissions==1 && submitted[0][1]==0 && submitted[0][17]==1);
    } else if (!strcmp(argv[1], "hid-control-reserve")) {
        k_timer_stop(&event_timer);
        for (unsigned i=0; i<MAX_REPORTS; i++) publish(i, 0);
        assert(total_dropped_reports==HID_FIFO_PRIORITY_RESERVE);
        publish(90,RCV_HID_TYPE_CMD_ACK); publish(91,240);
        assert(total_dropped_reports==HID_FIFO_PRIORITY_RESERVE);
        advance(2); while (!hid_fifo_is_empty()) { done(); advance(2); }
        assert(submissions==MAX_REPORTS/4);
        assert(submitted[submissions-1][2*16]==RCV_HID_TYPE_CMD_ACK);
        assert(submitted[submissions-1][3*16]==240);
    } else if (!strcmp(argv[1], "hid-frame-backlog")) {
        k_timer_stop(&event_timer);
        for (unsigned i=0;i<12;i++) publish(i,240);
        advance(2); assert(submissions==1);
        for (unsigned frame=0;frame<8;frame++) {
            advance(1); /* Host completes one transfer per frame. */
            for (unsigned i=0;i<4;i++) publish(12+frame*4+i,240);
            unsigned before=submissions; done(); advance(0);
            /* Completion must pull an already-armed producer deadline forward,
             * rather than holding the accumulated backlog for another frame. */
            assert(submissions==before+1);
        }
    } else if (!strcmp(argv[1], "hid-registration")) {
        stored_trackers=1; publish(1,240); advance(2); assert(submissions==1); done();
        int64_t sent_at=last_registration_sent; advance(98); assert(submissions==1);
        advance(3); assert(submissions==2); assert(last_registration_sent-sent_at<=101); assert(submitted[1][0]==255);
    } else if (!strcmp(argv[1], "hid-event-stale-retry")) {
        k_timer_stop(&event_timer);
        uint8_t expected[16]; event_record(expected);
        failures=1; advance(1);
        assert(attempts==1 && submissions==0 && !hid_fifo_is_empty());
        tracker_events_usb_reset();
        advance(1); assert(submissions==1 && hid_fifo_is_empty());
        for (unsigned i=0;i<4;i++) assert(submitted[0][i*16]==0xf8);
    } else if (!strcmp(argv[1], "hid-event-valid-retry")) {
        k_timer_stop(&event_timer);
        uint8_t expected[16]; event_record(expected);
        failures=1; advance(1);
        assert(submissions==0 && !hid_fifo_is_empty());
        advance(1); assert(submissions==1 && hid_fifo_is_empty());
        assert(!memcmp(submitted[0],expected,16));
    } else if (!strcmp(argv[1], "hid-event-classification")) {
        k_timer_stop(&event_timer);
        uint8_t event[16]; uint32_t generation=event_record(event);
        assert(generation!=0);
        tracker_events_usb_reset();
        uint8_t ack[16]={251,0,225,0,0xff,1,2,3,4,5,6,7,8,9,10,0xab};
        hid_write_packet_n(ack,0x55);
        advance(1); assert(submissions==1);
        assert(submitted[0][0]==0xf8 && !memcmp(submitted[0]+16,ack,16));
        assert(!hid_type_byte1_is_tracker_id(251) && !hid_type_has_rssi_slot(251));
        assert(total_tracker_drops[0]==0);
    } else { assert(!"unknown HID case"); }
    puts(argv[1]); return 0;
}
'''

cdc_source = (src / 'data_collect.c').read_text()
cdc_body = cdc_source[cdc_source.index('LOG_MODULE_REGISTER'):]
cdc_crc = ''
try:
    function(cdc_body, 'crc8_ccitt')
except ValueError:
    zephyr_base = Path(os.environ.get(
        'ZEPHYR_BASE',
        (Path(__file__).resolve().parents[4] / 'sdk-nrf').resolve().parent / 'zephyr',
    ))
    crc_source = (zephyr_base / 'subsys/crc/crc8_sw.c').read_text()
    crc_table_start = crc_source.index('static const uint8_t crc8_ccitt_small_table')
    crc_table_end = crc_source.index('};', crc_table_start) + 2
    cdc_crc = '#define __weak\n' + crc_source[crc_table_start:crc_table_end] + '\n' + function(crc_source, 'crc8_ccitt')
cdc = common + r'''
#define UART_LINE_CTRL_DTR 1
#define ESB_PONG_FLAG_DATA_COLLECT_OFF 1
#define ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF 2
static void (*uart_callback)(const struct device *, void *);
static bool dtr=true, tx_enabled, uart_ready=true;
static unsigned space=65536;
static uint8_t output_bytes[131072];
static size_t output_len;
static bool zero_once;
static void (*disable_hook)(void);
static int uart_line_ctrl_get(const struct device *d, int cmd, uint32_t *v) { *v=dtr; return 0; }
static int uart_irq_callback_user_data_set(const struct device *d, void (*cb)(const struct device *,void*), void *ctx) { uart_callback=cb; return 0; }
static int uart_irq_update(const struct device *d) { return 1; }
static int uart_irq_is_pending(const struct device *d) { return tx_enabled && space>0; }
static int uart_irq_tx_ready(const struct device *d) { return tx_enabled && space>0; }
static void uart_irq_tx_enable(const struct device *d) { tx_enabled=true; }
static void uart_irq_tx_disable(const struct device *d) { if (disable_hook) { void (*fn)(void)=disable_hook; disable_hook=NULL; fn(); } tx_enabled=false; }
static int uart_fifo_fill(const struct device *d, const uint8_t *p, int n) {
    if (zero_once) { zero_once=false; return 0; }
    unsigned count=MIN((unsigned)n,space); assert(output_len+count<=sizeof(output_bytes));
    memcpy(output_bytes+output_len,p,count); output_len+=count; space-=count; return (int)count;
}
static void sys_put_be32(uint32_t v, uint8_t *p) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }
static unsigned off_commands;
static void esb_send_remote_command(uint8_t id, int flag) { off_commands++; }
void data_collect_stop(void);
void data_collect_batch_stop(void);
''' + cdc_crc + cdc_body + r'''
static const uint8_t payload[]={0x12,3,0x55,0xaa,0x19};
static void publish(void) { data_collect_write(payload,sizeof(payload),0x91); }
static void irq(void) { if (uart_ready && tx_enabled && space) uart_callback(&device,NULL); }
static void drain(unsigned n) { for (unsigned i=0;i<n;i++) { advance(1); irq(); } }
static void publish_before_disable(void) { publish(); advance(0); /* Kick wins before the stale disable. */ }
static void verify_frame(size_t offset) {
    assert(output_len>=offset+sizeof(payload)+9);
    assert(output_bytes[offset]==0xaa && output_bytes[offset+1]==0x55);
    assert(output_bytes[offset+2]==sizeof(payload)+5);
    assert(!memcmp(output_bytes+offset+3,payload,sizeof(payload)));
    assert(output_bytes[offset+3+sizeof(payload)]==0x91);
    assert(crc8_ccitt(0x07,output_bytes+offset+2,sizeof(payload)+6)==output_bytes[offset+sizeof(payload)+8]);
}
int main(int argc, char **argv) {
    assert(argc==2); assert(data_collect_init()==0);
    if (!strcmp(argv[1],"cdc-idle")) {
        advance(1000); assert(dc_tx_kick_work.runs==0 && output_len==0);
    } else if (!strcmp(argv[1],"cdc-full-zero")) {
        publish(); space=0; advance(4); irq(); assert(output_len==0 && buf_used()==sizeof(payload)+9);
        space=5; zero_once=true; drain(1); assert(output_len==0); drain(1); assert(output_len==5);
        space=65536; drain(3); assert(buf_used()==0 && output_len==sizeof(payload)+9); verify_frame(0);
        unsigned before=dc_tx_kick_work.runs; advance(100); assert(dc_tx_kick_work.runs==before);
    } else if (!strcmp(argv[1],"cdc-disable-race")) {
        publish(); advance(0); disable_hook=publish_before_disable; irq(); drain(3);
        assert(output_len==2*(sizeof(payload)+9)); assert(buf_used()==0); verify_frame(0); verify_frame(sizeof(payload)+9);
    } else if (!strcmp(argv[1],"cdc-close-reopen")) {
        publish(); space=0; advance(1); dtr=false; advance(2); assert(buf_used()==0 && output_len==0);
        dtr=true; publish(); uart_ready=false; space=65536; drain(3); assert(output_len==0);
        uart_ready=true; drain(3); assert(buf_used()==0 && output_len==sizeof(payload)+9); verify_frame(0);
    } else if (!strcmp(argv[1],"cdc-stop-start")) {
        data_collect_start(3); publish(); space=0; advance(1); data_collect_stop();
        data_collect_batch_start(BIT(3),400); publish(); data_collect_batch_stop();
        space=65536; drain(3); assert(output_len==2*(sizeof(payload)+9) && buf_used()==0); verify_frame(0); verify_frame(sizeof(payload)+9);
        data_collect_start(3); advance(61000); assert(!data_collect_is_active() && off_commands==1);
    } else if (!strcmp(argv[1],"cdc-wrap")) {
        dc_buf_head=DATA_COLLECT_BUF_SIZE-4; dc_buf_tail=dc_buf_head;
        publish(); drain(3); assert(buf_used()==0 && output_len==sizeof(payload)+9); verify_frame(0);
    } else { assert(!"unknown CDC case"); }
    puts(argv[1]); return 0;
}
'''

cases = ['hid-idle', 'hid-producer-completion', 'hid-retry', 'hid-ready', 'hid-reserved-head', 'hid-control-reserve', 'hid-registration', 'hid-frame-backlog',
         'hid-event-stale-retry', 'hid-event-valid-retry', 'hid-event-classification',
         'cdc-idle', 'cdc-full-zero', 'cdc-disable-race', 'cdc-close-reopen', 'cdc-stop-start', 'cdc-wrap']
selected = args.case or cases
with tempfile.TemporaryDirectory(prefix='receiver-wake-smoke-') as directory:
    temp = Path(directory)
    spec=importlib.util.spec_from_file_location('event_host_runner',Path(__file__).resolve().parents[1]/'tracker_events/run.py')
    event_runner=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(event_runner)
    (temp/'zephyr/logging').mkdir(parents=True)
    (temp/'zephyr/kernel.h').write_text(event_runner.KERNEL)
    (temp/'zephyr/logging/log.h').write_text(event_runner.LOGGING)
    binaries = {}
    for name, content in [('hid',hid),('cdc',cdc)]:
        if not any(case.startswith(name+'-') for case in selected):
            continue
        cpath=temp/(name+'.c'); cpath.write_text(content)
        binary=temp/name
        event_sources=[str(src/'tracker_events.c')] if name=='hid' else []
        subprocess.run(shlex.split(os.environ.get('CC','cc'))+[
            '-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-unused-function','-Wno-unused-variable',
            '-Wno-misleading-indentation','-Wno-unused-but-set-variable',
            '-I'+str(temp),'-I'+str(src),
            '-g','-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie',str(cpath),*event_sources,'-o',str(binary)], check=True)
        binaries[name]=binary
    failed=[]
    for case in selected:
        result=subprocess.run([str(binaries[case.split('-')[0]]),case])
        if result.returncode:
            failed.append(case)
    if failed:
        raise SystemExit('Failed: '+', '.join(failed))
