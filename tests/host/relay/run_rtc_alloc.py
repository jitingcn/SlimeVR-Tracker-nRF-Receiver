from pathlib import Path
import subprocess,tempfile
h=Path(__file__).resolve().parent
s=(h.parents[2]/'src/connection/esb.c').read_text()
a=s.index('\tint rtc_channel = z_nrf_rtc_timer_chan_alloc();',s.index('static void relay_control_thread'))
b=s.index('\tuint32_t sent =',a)
c='''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
static bool relay_control_ready;
static int calls,fail,freed,relay_control_err;
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned k){(void)k;}
static int z_nrf_rtc_timer_compare_int_lock(int c){return c;}
static unsigned z_nrf_rtc_timer_compare_evt_address_get(int c){return c;}
#define LOG_ERR(...) ((void)0)
static int z_nrf_rtc_timer_chan_alloc(void){return ++calls==fail?-12:calls;}
static void z_nrf_rtc_timer_chan_free(int c){assert(c==1);freed++;}
static void initialize(void){'''+s[a:b]+'''(void)freeze_channel;(void)launch_event;}
int main(void){fail=1;initialize();assert(!relay_control_ready && calls==1 && !freed && relay_control_err==-12);
calls=0;fail=2;initialize();assert(!relay_control_ready && calls==2 && freed==1 && relay_control_err==-12);
calls=freed=0;fail=0;initialize();assert(relay_control_ready && calls==2 && !freed);}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'t.c';p.write_text(c);exe=Path(t)/'t'
 subprocess.run(['cc','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('RTC allocation: first failure, second failure cleanup, successful lifetime ownership passed')
