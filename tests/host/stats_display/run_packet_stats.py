#!/usr/bin/env python3
"""Reproduce a packet rollover between the monitor's sample and its update."""
import argparse, os, pathlib, shlex, subprocess, sys, tempfile
here=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'harness'))
from c_extract import extract_block
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',type=pathlib.Path,default=here.parents[2]/'src/connection/esb.c')
source=parser.parse_args().source.read_text()
def function(name):
 return extract_block(source,rf'^static void {name}\([^;{{]*\)\s*\{{')
if 'static void packet_stats_count(' in source:
 count=function('packet_stats_count')
else:
 sequence=extract_block(source,r'^static int check_packet_sequence\([^;{]*\)\s*\{')
 start=sequence.index('\tuint64_t current_time')
 end=sequence.index('\n\t// First packet',start)
 count='static void packet_stats_count(struct packet_stats *stats) {\n'+sequence[start:end]+'\n}'

if 'static void packet_stats_monitor(' in source:
 implementation=function('packet_stats_refresh')+'\n'+count+'\n'+function('packet_stats_monitor')
else:
 # Preserve the old monitor's sampled time and actual loop for regression.
 thread=function('esb_stats_thread')
 old=thread.split('// Update TPS for each tracker',1)[1].split('// Check if detailed stats',1)[0]
 implementation=count+'\nstatic void packet_stats_monitor(struct packet_stats *target) {\nuint64_t now=monitor_sample; tracker_slots[0].stats=*target;\n'+old+'\n*target=tracker_slots[0].stats; }'
fixture=r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define TPS_CALCULATION_INTERVAL_MS 1000
#define MAX_TRACKERS 1
struct packet_stats { uint64_t last_tps_time,last_packet_time; uint32_t current_tps,packets_in_last_second; };
static struct { struct packet_stats stats; } tracker_slots[1];
static uint64_t clock_ms,monitor_sample;
static unsigned locked,clock_reads_locked;
static unsigned irq_lock(void) { unsigned old=locked;locked=1;return old; }
static void irq_unlock(unsigned key) { locked=key; }
static uint64_t k_uptime_get(void) { clock_reads_locked+=locked;return clock_ms; }
'''
tests=r'''
int main(void) {
 (void)tracker_slots;(void)irq_lock;(void)irq_unlock;(void)monitor_sample;
 struct packet_stats s={.last_tps_time=1,.packets_in_last_second=120};
 monitor_sample=1000;clock_ms=1001;
 packet_stats_count(&s); /* RX rolled the window after monitor_sample. */
 assert(s.current_tps==120 && s.packets_in_last_second==1 && s.last_tps_time==1001);
 packet_stats_monitor(&s);
 assert(s.current_tps==120 && s.packets_in_last_second==1 && s.last_tps_time==1001);
 assert(clock_reads_locked==2 && !locked);
 clock_ms=2001;packet_stats_monitor(&s);assert(s.current_tps==1 && !s.packets_in_last_second);
 clock_ms=3001;packet_stats_monitor(&s);assert(!s.current_tps);
 s=(struct packet_stats){0};clock_ms=42;packet_stats_monitor(&s);assert(!s.last_tps_time);
 for(unsigned i=0;i<127;i++)packet_stats_count(&s);
 assert(s.packets_in_last_second==127 && s.last_tps_time==42 && s.last_packet_time==42);
 clock_ms=1042;packet_stats_monitor(&s);assert(s.current_tps==127 && !locked);
 s=(struct packet_stats){.last_tps_time=UINT64_C(0xffffffff)-500,.packets_in_last_second=120};
 clock_ms=s.last_tps_time+999;packet_stats_monitor(&s);assert(s.packets_in_last_second==120);
 clock_ms++;packet_stats_monitor(&s);assert(s.current_tps==120 && !s.packets_in_last_second);
 puts("TPS windows: stale monitor sample, atomic rollover, idle decay and 64-bit uptime passed");
}
'''
with tempfile.TemporaryDirectory() as temp:
 c=pathlib.Path(temp)/'test.c';exe=pathlib.Path(temp)/'test'
 c.write_text(fixture+'\n'+implementation+'\n'+tests)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-O1','-g','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
