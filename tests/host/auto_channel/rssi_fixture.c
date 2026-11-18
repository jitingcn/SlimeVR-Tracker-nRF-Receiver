#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include "rssi_scan.h"
#include "rssi_scan_fem.h"
#define __aligned(x) __attribute__((aligned(x)))
#define CONFIG_RADIO_RF_CHANNEL 18
#define RADIO_STATE_STATE_Disabled 0
#define RADIO_SHORTS_END_START_Msk 0x20
static struct {
 uint32_t STATE,FREQUENCY,SHORTS,EVENTS_READY,EVENTS_RSSIEND,EVENTS_DISABLED,EVENTS_END;
 uint32_t TASKS_RXEN,TASKS_START,TASKS_RSSISTART,TASKS_DISABLE,RSSISAMPLE;
 uint32_t PCNF1,RXADDRESSES,EVENTS_ADDRESS;
 uintptr_t PACKETPTR;
} hw;
#define NRF_RADIO (&hw)
static uint32_t tick;
static unsigned visits[101], sample_count, ends, reanchors, releases, starts;
static bool owner, fail_ready, fail_rssi, fail_disable, fail_restore, packet_end;
static bool fail_one_visit, fail_all_visits, preempt;
static uint8_t physical_sample, home;
static char text[16384];
static size_t text_len;
static int printk(const char *fmt, ...) {
 va_list ap; va_start(ap,fmt); int n=vsnprintf(text+text_len,sizeof(text)-text_len,fmt,ap); va_end(ap);
 assert(n>=0 && (size_t)n<sizeof(text)-text_len); text_len+=(size_t)n; return n;
}
/* Physical microseconds and RTC cycles are separate: one RTC tick is 30.5 us. */
static uint64_t rtc_epoch_us;
static volatile uint32_t *scheduled_event;
static uint64_t completion_us;
static unsigned clock_reads, interrupt_read, boundary_interrupts;
static bool suppress_completion;
static void advance_boundary_hardware(void) {
 if (scheduled_event && !suppress_completion && rtc_epoch_us+tick>=completion_us)
  *scheduled_event=1;
}
static uint32_t k_cycle_get_32(void) {
 if (scheduled_event && ++clock_reads==interrupt_read) {
  tick+=1000; /* IRQ may fall between the poll's event and elapsed-time reads. */
  boundary_interrupts++;
  advance_boundary_hardware(); /* RADIO keeps running while the thread is stopped. */
 }
 return (uint32_t)((rtc_epoch_us+tick)*32768u/1000000u);
}
static uint32_t k_cyc_to_us_floor32(uint32_t n) { return (uint32_t)((uint64_t)n*1000000u/32768u); }
static uint32_t k_us_to_cyc_ceil32(uint32_t n) { return ((uint64_t)n*32768u+999999u)/1000000u; }
static void k_busy_wait(unsigned us) {
 tick+=us;
 advance_boundary_hardware();
 if (hw.TASKS_RXEN) { hw.TASKS_RXEN=0; hw.STATE=1; visits[hw.FREQUENCY]++; if (!fail_ready) { hw.STATE=2; hw.EVENTS_READY=1; } }
 if (hw.TASKS_START) { hw.TASKS_START=0; if(hw.STATE==2) hw.STATE=3; }
 if (hw.TASKS_RSSISTART) {
  hw.TASKS_RSSISTART=0;
  if(packet_end && !ends) { ends++; hw.EVENTS_END=1; hw.STATE=(hw.SHORTS & RADIO_SHORTS_END_START_Msk)?3:2; }
  bool bad=fail_rssi || fail_all_visits || (fail_one_visit && hw.FREQUENCY==18 && visits[18]==1);
  if(!bad && hw.STATE==3) { hw.EVENTS_RSSIEND=1; hw.RSSISAMPLE=physical_sample; sample_count++; }
  if(preempt) tick+=1000;
 }
 if(hw.TASKS_DISABLE) { hw.TASKS_DISABLE=0; if(!fail_disable) { hw.STATE=0; hw.EVENTS_DISABLED=1; } }
}
static void k_msleep(unsigned ms) { tick+=ms*1000; }
static void k_yield(void) {}
static int esb_radio_operation_begin(void) { assert(!owner); owner=true; return 0; }
static void esb_radio_operation_end(void) { assert(owner && hw.STATE==3); releases++; owner=false; }
static int esb_radio_operation_stop(void) { assert(owner); hw.STATE=0; return 0; }
static void esb_set_addr_unified(void) { assert(owner); }
static int esb_radio_quiesce(void) { assert(owner); if(fail_disable) return -EIO; hw.STATE=0; return 0; }
static int esb_initialize_quiet(bool tx) { assert(!tx && owner && hw.STATE==0); return fail_restore ? -EIO : 0; }
static int esb_start_rx(void) { starts++; hw.STATE=3; return 0; }
static void esb_rx_discontinuity(void) { assert(owner && hw.STATE==0); reanchors++; }
static uint8_t esb_get_receiver_channel(void) { return home; }
static bool esb_channel_is_allowed(uint8_t c) { return c<=88 && !(c&1) && c!=4 && c!=22 && c!=26 && c!=80; }
/* ACTUAL_SOURCE */
static void reset(void) {
 memset(&hw,0,sizeof(hw)); memset(visits,0,sizeof(visits)); memset(text,0,sizeof(text));
 tick=sample_count=ends=reanchors=releases=starts=0; text_len=0;
 owner=fail_ready=fail_rssi=fail_disable=fail_restore=packet_end=fail_one_visit=fail_all_visits=false; physical_sample=96;
 preempt=false; home=18;
 hw.PCNF1=72;
}
static int boundary_cases(void) {
 unsigned failures=0, cases=0;
 const char *names[]={"READY","RSSIEND","DISABLED"};
 for (unsigned stage=0;stage<3;stage++) {
  for (unsigned wrap=0;wrap<2;wrap++) {
   for (unsigned phase=0;phase<31;phase++) {
    for (unsigned missing=0;missing<2;missing++) {
     for (unsigned injection=0;injection<3;injection++) {
      reset();
      /* 131072 seconds is exactly one 32-bit 32768-Hz cycle wrap. */
      rtc_epoch_us=(wrap ? UINT64_C(131072000000)-100 : 0)+phase;
      scheduled_event=stage==0 ? &hw.EVENTS_READY : stage==1 ? &hw.EVENTS_RSSIEND : &hw.EVENTS_DISABLED;
      completion_us=rtc_epoch_us+40;
      clock_reads=boundary_interrupts=0;
      /* No IRQ, IRQ before first event observation, or IRQ at expiry read. */
      interrupt_read=injection;
      suppress_completion=missing;
      bool ok=wait_for_event_and_clear(scheduled_event,200);
      bool expected=!missing;
      if (ok!=expected || *scheduled_event!=(unsigned)(!missing && !ok) ||
          (injection && boundary_interrupts!=1) || (!injection && tick>214)) {
       if (failures<6) printf("boundary %s wrap=%u phase=%u missing=%u clock_hook=%u: success=%u event=%u elapsed_us=%u\n",
                             names[stage],wrap,phase,missing,injection,ok,*scheduled_event,tick);
       failures++;
      }
      cases++;
      scheduled_event=NULL;
     }
    }
   }
  }
 }
 rtc_epoch_us=0;
 printf("RSSI RTC boundary: %u cases, %u failures\n",cases,failures);
 return failures ? 1 : 0;
}
int main(void) {
 assert(boundary_cases()==0);
 reset();
 struct rssi_scan_observation sample={.peak=123};
 /* Every RTC phase completes equal bins, including counter wrap. */
 for(unsigned wrap=0;wrap<2;wrap++) for(unsigned phase=0;phase<31;phase++) {
  reset(); rtc_epoch_us=(wrap?UINT64_C(131072000000)-100:0)+phase;
  assert(rssi_scan_measure_bounded(18,&sample)==0 && sample.peak==96);
  assert(sample.samples==RSSI_SCAN_SLICE_US/RSSI_SCAN_SLOT_US);
  uint32_t minimum=k_cyc_to_us_floor32(k_us_to_cyc_ceil32(RSSI_SCAN_SLOT_US)*sample.samples-1);
  assert(sample.coverage_us==minimum && tick>=minimum && tick<1100);
 }
 rtc_epoch_us=0;
 reset(); physical_sample=0; assert(rssi_scan_measure_bounded(18,&sample)==0 && sample.peak==0);
 reset(); fail_rssi=true; sample.peak=123; assert(rssi_scan_measure_bounded(18,&sample)==-ETIMEDOUT && sample.peak==123);
 reset(); preempt=true; assert(rssi_scan_measure_bounded(18,&sample)==-ETIMEDOUT);
 reset(); preempt=true; fail_rssi=true; assert(rssi_scan_measure_bounded(18,&sample)==-ETIMEDOUT);
 reset(); hw.EVENTS_READY=1; fail_ready=true; assert(rssi_scan_measure_bounded(18,&sample)==-ETIMEDOUT);
 reset(); hw.STATE=3; assert(rssi_scan_measure_bounded(18,&sample)==-EBUSY);
 assert(rssi_scan_measure_bounded(101,&sample)==-EINVAL && rssi_scan_measure_bounded(18,NULL)==-EINVAL);
 reset(); fail_ready=true; assert(rssi_scan_measure_bounded(18,&sample)==-ETIMEDOUT && hw.STATE==0);
 reset(); fail_disable=true; assert(rssi_scan_measure_bounded(18,&sample)<0 && hw.STATE!=0);
 reset(); hw.EVENTS_READY=hw.EVENTS_RSSIEND=1; fail_rssi=true; assert(rssi_scan_measure_bounded(18,&sample)==-ETIMEDOUT);
 reset(); packet_end=true; hw.SHORTS=0x400; assert(rssi_scan_measure_bounded(18,&sample)==0 && sample.peak==96 && ends==1 && hw.SHORTS==0x400);
 reset(); fail_one_visit=true; rssi_scan_run_and_print();
 assert(visits[18]==200 && !visits[26] && releases==1 && reanchors==2);
 assert(scan_results[18].score==-1 && scan_results[18].failures==1 && scan_results[16].score==96 && !scan_results[16].failures);
 assert(!strstr(text,"18 (- 0 dBm)") && !strstr(text,"Recommended: channel 18"));
 assert(strstr(text,"invalid"));
 reset(); fail_all_visits=true; rssi_scan_run_and_print();
 assert(sample_count==0 && !strstr(text,"Recommended: channel") && strstr(text,"no valid"));
 for(unsigned ch=0;ch<=88;ch++) if(esb_channel_is_allowed(ch)) assert(scan_results[ch].score==-1 && scan_results[ch].failures==200);
 reset(); physical_sample=0; rssi_scan_run_and_print(); assert(strstr(text,"Recommended: channel 18 (-0 dBm)"));
 reset(); fail_restore=true; rssi_scan_run_and_print(); assert(owner && !releases && !starts && !strstr(text,"Recommended:"));
 reset(); fail_disable=true; rssi_scan_run_and_print(); assert(owner && !releases && !starts);
 for(unsigned ch=0;ch<=100;ch++) { reset(); home=ch; assert(get_current_effective_channel()==ch); }
 reset(); home=255; assert(get_current_effective_channel()==CONFIG_RADIO_RF_CHANNEL);
 puts("RSSI events/manual: valid zero, timeout, stale-event rejection, END_START, strict channel invalidation, all-invalid, restore fault passed");
 return 0;
}
