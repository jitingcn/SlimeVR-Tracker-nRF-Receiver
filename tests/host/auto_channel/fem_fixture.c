#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <assert.h>
#define __aligned(x) __attribute__((aligned(x)))
#define BIT(n) (1U << (n))
#define CONFIG_RADIO_RF_CHANNEL 18
#define RADIO_STATE_STATE_Disabled 0
#define RADIO_SHORTS_END_START_Msk 0x20
static struct {
 uint32_t STATE,FREQUENCY,SHORTS,RXADDRESSES,PCNF1;
 uint32_t EVENTS_READY,EVENTS_RSSIEND,EVENTS_DISABLED,EVENTS_END,EVENTS_ADDRESS;
 uint32_t TASKS_RXEN,TASKS_START,TASKS_RSSISTART,TASKS_DISABLE,RSSISAMPLE;
 uintptr_t PACKETPTR;
} hw;
#define NRF_RADIO (&hw)
static uint32_t now;
static bool fail_ready,fail_rssi,fail_disable,owner;
static unsigned samples,restores,starts;
#if TEST_FEM
#define CONFIG_MPSL_FEM 1
#define CONFIG_SOC_SERIES_NRF52 1
#define CONFIG_ESB_SYS_TIMER_INSTANCE 2
#define DT_NODE_HAS_PROP(...) 0
#define NRF_TIMER_EVENT_COMPARE2 2
#define NRF_TIMER_EVENT_COMPARE3 3
#define NRF_TIMER_CC_CHANNEL3 3
#define NRF_TIMER_SHORT_COMPARE3_STOP_MASK 1
#define NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK 2
#define NRF_RADIO_EVENT_DISABLED 0
#define MPSL_FEM_EVENT_TYPE_TIMER 0
#define MPSL_FEM_EVENT_TYPE_GENERIC 1
#define MPSL_FEM_ALL 3
static struct { uint32_t cc[4],events[4],shorts,elapsed; bool running; } timer;
#define NRF_TIMER_INST_GET(id) (&timer)
typedef struct { __typeof__(&timer) p_reg; } nrfx_timer_t;
typedef struct { unsigned frequency; } nrfx_timer_config_t;
#define NRFX_TIMER_INSTANCE(reg) {.p_reg=(reg)}
#define NRFX_TIMER_DEFAULT_CONFIG(hz) {.frequency=(hz)}
typedef struct {
 int type;
 union {
  struct { __typeof__(&timer) p_timer_instance; struct { uint32_t start,end; } counter_period; unsigned compare_channel_mask; } timer;
  struct { uintptr_t event; } generic;
 } event;
} mpsl_fem_event_t;
static bool initialized,enabled,configured,lna,fail_init,fail_config,fail_clear,fail_powerdown,freeze_timer;
static unsigned inits,uninits,activated,forced_off;
static uint32_t lna_since;
static int nrfx_timer_init(nrfx_timer_t *t,const nrfx_timer_config_t *c,void *cb) {
 assert(t->p_reg==&timer && c->frequency==1000000 && !cb && !initialized);
 if(fail_init) return -EIO;
 initialized=true; inits++; return 0;
}
static void nrfx_timer_uninit(nrfx_timer_t *t) { (void)t; assert(initialized); initialized=false; uninits++; }
static void nrfx_timer_enable(nrfx_timer_t *t) { (void)t; timer.running=true; }
static void nrfx_timer_disable(nrfx_timer_t *t) { (void)t; timer.running=false; }
static void nrfx_timer_clear(nrfx_timer_t *t) { (void)t; timer.elapsed=0; }
#define nrf_timer_shorts_set(t,v) ((t)->shorts=(v))
#define nrf_timer_event_clear(t,n) ((t)->events[n]=0)
#define nrf_timer_event_check(t,n) ((t)->events[n]!=0)
#define nrf_timer_cc_set(t,n,v) ((t)->cc[n]=(v))
static uintptr_t nrf_radio_event_address_get(void *radio,int event) { (void)radio; (void)event; return (uintptr_t)&hw.EVENTS_DISABLED; }
static void mpsl_fem_enable(void) { enabled=true; }
static int mpsl_fem_lna_configuration_set(const mpsl_fem_event_t *a,const mpsl_fem_event_t *d) {
 assert(enabled && !lna && a->type==MPSL_FEM_EVENT_TYPE_TIMER);
 assert(a->event.timer.p_timer_instance==&timer && a->event.timer.compare_channel_mask==BIT(2));
 assert(d->type==MPSL_FEM_EVENT_TYPE_GENERIC && d->event.generic.event==(uintptr_t)&hw.EVENTS_DISABLED);
 configured=true; timer.cc[2]=a->event.timer.counter_period.end-8;
 /* Configuration installs routing; only a running TIMER can assert CRX. */
 return fail_config ? -EIO : 0;
}
static int mpsl_fem_lna_configuration_clear(void) { if(fail_clear) return -EIO; configured=false; return 0; }
static void mpsl_fem_deactivate_now(int mask) { assert(mask==MPSL_FEM_ALL); lna=false; forced_off++; }
static int mpsl_fem_disable(void) { if(fail_powerdown || configured) return -EIO; enabled=false; return 0; }
#endif
static uint32_t k_cycle_get_32(void) { return now; }
static uint32_t k_cyc_to_us_floor32(uint32_t n) { return n; }
static uint32_t k_us_to_cyc_ceil32(uint32_t n) { return n; }
#if TEST_FEM
static bool delay_worker;
#endif
static void k_busy_wait(unsigned us) {
#if TEST_FEM
 if(delay_worker && timer.running) { us+=300; delay_worker=false; }
#endif
 for(unsigned i=0;i<us;i++) {
  now++;
#if TEST_FEM
  if(timer.running && !freeze_timer) {
   timer.elapsed++;
   if(timer.elapsed==timer.cc[2] && configured && enabled) { lna=true; lna_since=now; activated++; }
   if(timer.elapsed==timer.cc[3]) {
    timer.events[3]=1;
    if(timer.shorts & NRF_TIMER_SHORT_COMPARE3_STOP_MASK) timer.running=false;
    if(timer.shorts & NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK) timer.elapsed=0;
   }
  }
#endif
  if(hw.TASKS_RXEN) {
   hw.TASKS_RXEN=0;
#if TEST_FEM
   assert(lna && now-lna_since>=8);
#endif
   hw.STATE=1; if(!fail_ready) { hw.STATE=2; hw.EVENTS_READY=1; }
  }
  if(hw.TASKS_START) { hw.TASKS_START=0; if(hw.STATE==2) hw.STATE=3; }
  if(hw.TASKS_RSSISTART) {
   hw.TASKS_RSSISTART=0;
#if TEST_FEM
   assert(lna && now-lna_since>=8);
#endif
   /* Packet completion must restart RX without deactivating the LNA. */
   hw.EVENTS_END=1;
   hw.STATE=(hw.SHORTS & RADIO_SHORTS_END_START_Msk) ? 3 : 2;
   if(!fail_rssi && hw.STATE==3) { hw.EVENTS_RSSIEND=1; hw.RSSISAMPLE=84; samples++; }
  }
  if(hw.TASKS_DISABLE) { hw.TASKS_DISABLE=0; if(!fail_disable) {
   hw.STATE=0; hw.EVENTS_DISABLED=1;
#if TEST_FEM
   if(configured) lna=false;
#endif
  } }
 }
}
static void k_msleep(unsigned ms) { k_busy_wait(ms*1000); }
static void k_yield(void) {}
static int printk(const char *fmt,...) { (void)fmt; return 0; }
static int esb_radio_operation_begin(void) { assert(!owner); owner=true; return 0; }
static void esb_radio_operation_end(void) { assert(owner); owner=false; }
static int esb_radio_operation_stop(void) { assert(owner); hw.STATE=0; return 0; }
static void esb_set_addr_unified(void) { assert(owner); }
static int esb_radio_quiesce(void) { return fail_disable ? -EIO : 0; }
static void esb_rx_discontinuity(void) {}
static uint8_t esb_get_receiver_channel(void) { return 18; }
static bool esb_channel_is_allowed(uint8_t ch) { return ch==18; }
/* FEM_HEADER */
static int esb_initialize_quiet(bool tx) {
 assert(!tx && rssi_scan_fem_is_safe());
#if TEST_FEM
 assert(!initialized && !timer.running && !configured && !lna);
#endif
 restores++; return 0;
}
static int esb_start_rx(void) { starts++; hw.STATE=3; return 0; }
/* FEM_SOURCE */
/* SCAN_HEADER */
/* SCAN_SOURCE */
static void clean(void) {
 assert(rssi_scan_fem_is_safe());
#if TEST_FEM
 assert(!initialized && !timer.running && !configured && !lna && inits==uninits);
 unsigned old=activated; k_busy_wait(1000); assert(activated==old);
#endif
}
int main(int argc, char **argv) {
 assert(argc==2);
 (void)argv;
 struct rssi_scan_observation out;
 for(unsigned i=0;i<100;i++) {
  memset(&out,0,sizeof(out));
  assert(rssi_scan_measure_bounded(18,&out)==0);
  assert(out.peak==84 && out.quiet_floor==84 && out.samples==RSSI_SCAN_SLICE_US/RSSI_SCAN_SLOT_US);
  assert(out.coverage_us==RSSI_SCAN_SLICE_US-1);
  clean();
 }
#if TEST_FEM
 assert(activated==100);
 delay_worker=true;
 assert(rssi_scan_measure_bounded(18,&out)==0 && out.peak==84); clean();
 freeze_timer=true; out.peak=123;
 assert(rssi_scan_measure_bounded(18,&out)<0 && out.peak==123); freeze_timer=false; clean();
 fail_config=true; assert(rssi_scan_measure_bounded(18,&out)<0 && out.peak==123); fail_config=false; clean();
 fail_init=true; assert(rssi_scan_measure_bounded(18,&out)<0 && out.peak==123); fail_init=false; clean();
#endif
 fail_ready=true; out.peak=123; assert(rssi_scan_measure_bounded(18,&out)<0 && out.peak==123); fail_ready=false; clean();
 fail_rssi=true; assert(rssi_scan_measure_bounded(18,&out)<0 && out.peak==123); fail_rssi=false; clean();
 rssi_scan_run_and_print(); assert(!owner && restores==1 && starts==1); hw.STATE=0; clean();
 fail_disable=true; out.peak=123; assert(rssi_scan_measure_bounded(18,&out)<0 && out.peak==123 && hw.STATE!=0);
#if TEST_FEM
 assert(!lna && !timer.running && !initialized);
#endif
 fail_disable=false; hw.STATE=0;
#if TEST_FEM
 /* Failure clearing installed routing cannot be disguised by a later no-op
  * release. Manual restoration must retain ownership even with RADIO off. */
 fail_clear=strcmp(argv[1],"clear")==0;
 fail_powerdown=!fail_clear;
 rssi_scan_run_and_print();
 assert(owner && restores==1 && starts==1 && !rssi_scan_fem_is_safe());
 assert(!lna && !timer.running && !initialized && forced_off>0);
 assert(rssi_scan_fem_release()==-EIO && rssi_scan_fem_prepare()==-EIO);
#endif
 puts(TEST_FEM ? "FEM timed backend/raw: settled LNA, repeated rearm, release, invalidation and sticky unsafe cleanup passed" : "Non-FEM actual raw hooks: timed scan and error behavior passed");
 return 0;
}
