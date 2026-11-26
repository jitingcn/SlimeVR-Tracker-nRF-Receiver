#!/usr/bin/env python3
"""Exercise the installed ESB PRX transaction, including its timeout cleanup."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'harness'))
from c_extract import extract_block
SDK = Path(os.environ.get('NCS_ROOT', HERE.parents[3])) / 'nrf/subsys/esb/esb.c'
s = SDK.read_text()
body = '\n'.join(extract_block(s, rf'^(?:static void|static int|int|uint8_t \*)\s*{name}\([^;{{]*\)\s*\{{')
                 for name in ('esb_prx_tx_prepare', 'esb_prx_tx_begin', 'on_radio_disabled_prx_tx', 'prx_tx_reset',
                              'prx_tx_isolate', 'prx_tx_fault', 'prx_tx_cancel', 'esb_prx_tx_arm_event', 'esb_prx_tx_wait'))
prefix = r'''
#include <assert.h>
#define CONFIG_SOC_SERIES_NRF52 1
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#define IS_ENABLED(x) (x)
#define CONFIG_ESB_MPSL_TIMESLOT 0
#define CONFIG_ESB_MAX_PAYLOAD_LENGTH 72
#define CONFIG_ESB_PIPE_COUNT 8
#define ESB_MODE_PRX 1
#define ESB_PROTOCOL_ESB_DPL 1
#define ESB_STATE_UNINITIALIZED 0
#define ESB_STATE_PRX 1
#define ESB_STATE_PRX_TX 2
#define ESB_STATE_PRX_SEND_ACK 3
#define NRF_RADIO_STATE_DISABLED 0
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define NRF_RADIO ((void *)0)
#define RADIO_NORMAL_SW_SHORTS 3
#define ESB_RADIO_EVENT_END 1
#define NRF_RADIO_EVENT_DISABLED 2
#define NRF_RADIO_INT_DISABLED_MASK 1
#define NRF_TIMER_CC_CHANNEL1 1
#define __DMB() ((void)0)
struct esb_payload {unsigned length,pipe;uint8_t data[72];};
struct esb_radio_pdu {struct {struct {uint8_t length,pid,ack;} dpl_pdu;} type;uint8_t data[72];};
static uint8_t prx_tx_buffer[80];
static uint8_t prx_tx_length, prx_tx_pipe, prx_tx_channel;
static int8_t prx_tx_power;
static bool prx_tx_begun, prx_tx_faulted;
static struct {struct {struct {unsigned compare_channel_mask; struct {unsigned end;} counter_period;} timer;} event;} tx_time_shifted;
static int disable_event;
#define TX_FAST_RAMP_UP_TIME_US 40
#define TX_RAMP_UP_TIME_US 130
#define NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK 1
#define NRF_TIMER_SHORT_COMPARE2_STOP_MASK 2
#define NRF_RADIO_TASK_DISABLE 1
static struct {int mode,protocol; int8_t tx_output_power; bool use_fast_ramp_up;} esb_cfg={.mode=1,.protocol=1,.use_fast_ramp_up=true};
static struct {uint8_t rf_channel;} esb_addr;
static struct {void *p_reg;} esb_timer;
static int esb_state=ESB_STATE_PRX;
static volatile bool prx_tx_active;
static void (*on_radio_disabled)(void);
static bool pa,ppi,timer,fail,late,stuck;
static bool prx_tx_event_fired(uint32_t event){(void)event;return late;}
static unsigned now,cc,restores,pipe,clears;
static void start_rx_listening(void){esb_state=ESB_STATE_PRX;restores++;}
static void esb_ppi_for_retransmission_clear(void){ppi=false;}
static void esb_fem_for_tx_retry_clear(void){pa=false;}
static void nrfx_timer_disable(void *p){(void)p;timer=false;}
static unsigned locked;
static unsigned irq_lock(void){unsigned key=locked;locked=1;return key;}
static void irq_unlock(unsigned key){locked=key;}
__attribute__((unused)) static void clear_peripherals(void){pa=ppi=timer=false;clears++;}
static void update_radio_tx_power(void){}
static void nrf_radio_txaddress_set(void *p,unsigned v){(void)p;pipe=v;}
static void nrf_radio_packetptr_set(void *p,void *v){(void)p;(void)v;}
static void nrf_radio_shorts_set(void *p,unsigned v){(void)p;(void)v;}
static void nrf_radio_event_clear(void *p,unsigned v){(void)p;(void)v;}
static void nrf_radio_int_enable(void *p,unsigned v){(void)p;(void)v;}


static void nrf_timer_cc_set(void *p,unsigned ch,unsigned n){(void)p;assert(ch==1);cc=n;}
static void k_busy_wait(unsigned n){assert(!locked);now+=n;if(!fail && now==cc+120)on_radio_disabled();}
'''
prefix += r"""
static void esb_ppi_for_prx_tx_disable_rx(void){ppi=false;}
static void nrf_radio_int_disable(void *p,unsigned v){(void)p;(void)v;}
static void nrf_radio_task_trigger(void *p,unsigned v){(void)p;(void)v;}
static bool nrf_radio_event_check(void *p,unsigned v){(void)p;(void)v;return !stuck;}
static unsigned nrf_radio_state_get(void *p){(void)p;return stuck;}
static void esb_ppi_for_txrx_clear(bool rx,bool start){(void)rx;(void)start;ppi=false;}
static void esb_fem_reset(void){pa=false;}
static bool irq_off;
static void esb_irq_disable(void){irq_off=true;}
static void errata_216_off(void){}
static void nrf_timer_shorts_set(void *p,unsigned v){(void)p;(void)v;}
static void nrf_timer_int_disable(void *p,unsigned v){(void)p;(void)v;}
static void mpsl_fem_lna_configuration_clear(void){}
static void mpsl_fem_utils_available_cc_channels_cache(unsigned v){(void)v;}
static int mpsl_fem_pa_configuration_set(const void *a,const void *b){(void)a;(void)b;pa=true;return 0;}
static void esb_ppi_for_prx_tx_set(uint32_t event){assert(event==1234);timer=false;ppi=true;}
"""
cases = r'''
int main(void){
 struct esb_payload p={.length=72,.pipe=1,.data={0xe8}};
 unsigned key=irq_lock();
 uint8_t *wire=esb_prx_tx_prepare(&p);
 assert(wire && wire != p.data);
 wire[20]=0xa5;
 assert(esb_prx_tx_begin()==0 && !timer && locked);
 assert(esb_prx_tx_arm_event(21,1234)==0 && locked && !timer);
 assert(prx_tx_active && ppi && pa && wire[20]==0xa5);
 /* No CPU polling launches the radio: the external event starts the timer. */
 timer=true;
 irq_unlock(key);
 assert(esb_prx_tx_wait(21)==0);
 now=0;fail=true;
 assert(esb_prx_tx_prepare(&p));
 key=irq_lock();assert(!esb_prx_tx_begin());
 assert(!esb_prx_tx_arm_event(21,1234));irq_unlock(key);
 assert(esb_prx_tx_wait(21)==-ETIMEDOUT);
 assert(restores==2 && !pa && !ppi && !timer && !prx_tx_active);
 now=0;fail=false;
 assert(esb_prx_tx_prepare(&p));
 key=irq_lock();
 assert(esb_prx_tx_begin()==0);
 late=true;
 assert(esb_prx_tx_arm_event(21,1234)==-ETIME);
 assert(esb_state==ESB_STATE_PRX && !pa && !ppi && !timer && !prx_tx_begun);
 irq_unlock(key);late=false;
 stuck=true;
 assert(esb_prx_tx_prepare(&p));assert(!esb_prx_tx_begin());
 assert(esb_prx_tx_arm_event(21,1234)==-ETIMEDOUT);
 assert(!prx_tx_begun && !ppi && esb_state==ESB_STATE_UNINITIALIZED);
 esb_state=ESB_STATE_PRX;
 assert(esb_prx_tx_prepare(&p));assert(!esb_prx_tx_begin());
 late=true;assert(esb_prx_tx_arm_event(21,1234)==-ETIMEDOUT);
 late=false;stuck=false;esb_state=ESB_STATE_PRX;
 prx_tx_length=0;assert(esb_prx_tx_begin()==-EBUSY);
 /* Preparation needs a listening PRX: it must not touch TXPOWER otherwise. */
 esb_state=9;assert(!esb_prx_tx_prepare(&p));
 assert(esb_prx_tx_begin()==-EBUSY);
 esb_state=ESB_STATE_UNINITIALIZED;assert(!esb_prx_tx_prepare(&p));
 esb_state=ESB_STATE_PRX;
 /* A budget near UINT32_MAX saturates instead of wrapping to zero polls. */
 now=0;assert(esb_prx_tx_prepare(&p));
 key=irq_lock();assert(!esb_prx_tx_begin());assert(!esb_prx_tx_arm_event(21,1234));irq_unlock(key);
 unsigned before=restores;int werr=esb_prx_tx_wait(UINT32_MAX-499U);
 assert(werr==0 && restores==before+1 && !prx_tx_active);
 /* RADIO never confirms DISABLED: keep RX/DMA isolated and report it. */
 now=0;fail=true;restores=0;
 assert(esb_prx_tx_prepare(&p));
 key=irq_lock();assert(!esb_prx_tx_begin());assert(!esb_prx_tx_arm_event(21,1234));irq_unlock(key);
 stuck=true;
 assert(esb_prx_tx_wait(21)==-EIO);
 assert(!restores && esb_state==ESB_STATE_UNINITIALIZED && prx_tx_faulted && irq_off);
 assert(!ppi && !pa && !prx_tx_active && !prx_tx_begun && !prx_tx_length);
 stuck=false;fail=false;prx_tx_faulted=false;irq_off=false;esb_state=ESB_STATE_PRX;
 esb_state=ESB_STATE_PRX;p.length=73;assert(!esb_prx_tx_prepare(&p));
 return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    src=Path(tmp)/'broadcast.c';exe=Path(tmp)/'broadcast'
    src.write_text(prefix+body+cases)
    subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror',
        '-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PRX TX success, late deadline, busy and timeout restoration: passed')
