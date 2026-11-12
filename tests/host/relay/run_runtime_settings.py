"""Execute receiver settings persistence and budget application from esb.c."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

from pathlib import Path
import os, shlex, subprocess, sys, tempfile
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'harness'))
from c_extract import extract_block
source = select_relay((HERE.parents[2] / 'src/connection/esb.c').read_text())
a = source.index('static struct relay_runtime_settings relay_settings =')
b = source.index('struct tdma_publication {', a)
state = source[a:b]
def function(name):
    return extract_block(source, rf'^(?:static )?(?:void|int) {name}\([^;{{]*\)\s*\{{')
prefix = r'''
#define CONFIG_RELAY_RUNTIME_LAYOUT 1
#define CONFIG_RELAY_MAX_TRACKERS 16
#define CONFIG_RELAY_MAX_REPAIRS 1
#define CONFIG_RELAY_REPAIR_FLOOR 0
#define CONFIG_RELAY_SLOT_TICKS 16
#define CONFIG_RELAY_TPS_MIN 100
#define CONFIG_RELAY_TPS_MAX 120
#define IS_ENABLED(x) (x)
#define K_FOREVER 0
#define LOG_WRN(...) ((void)0)
#define RELAY_LAYOUT_SETTINGS 261
#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "relay_protocol.h"
#ifdef CONFIG_RELAY_COORD_CAPACITY
#define RELAY_COORD_CAPACITY CONFIG_RELAY_COORD_CAPACITY
#else
#define RELAY_COORD_CAPACITY RELAY_MAX_TRACKERS
#endif
#include "relay_demand.h"
static unsigned mutex,depth,writes,recalculates,prints,wakes;
static struct relay_control relay_grants;
static uint8_t relay_profile;
static int tracker_store_lock,tdma_membership_wake,write_error,read_result=-ENOENT;
static uint8_t storage[10];
static unsigned irq_lock(void){assert(!depth);depth=1;return 0;}
static void irq_unlock(unsigned key){assert(!key && depth);depth=0;}
static void k_mutex_lock(int *p,int timeout){(void)p;(void)timeout;assert(!mutex);mutex=1;}
static void k_mutex_unlock(int *p){(void)p;assert(mutex);mutex=0;}
static void k_sem_give(int *p){(void)p;wakes++;}
static int printk(const char *format,...){(void)format;prints++;return 0;}
static int sys_read_checked(unsigned key,void *out,unsigned n){assert(key==261 && n==10);if(read_result==10 || read_result==9)memcpy(out,storage,read_result);return read_result;}
static int sys_write_checked(unsigned key,const void *data,unsigned n){assert(key==261 && n==10 && !depth && mutex);writes++;if(write_error)return write_error;memcpy(storage,data,n);return 0;}
static void tdma_recalculate(bool forced){assert(forced && mutex && !depth);recalculates++;}
'''
cases = r'''
static void defaults(void){
 relay_settings=(struct relay_runtime_settings){16,5,16,120,120,5, 0};
 relay_applied_settings=relay_settings;relay_settings_generation=1;relay_applied_generation=0;
 writes=recalculates=prints=wakes=0;write_error=0;read_result=-ENOENT;
}
static void saved(unsigned version,unsigned trackers,unsigned repairs,unsigned target,unsigned floor){
 storage[0]=version;storage[1]=trackers;storage[2]=repairs;storage[3]=16;
 relay_put_be16(storage+4,target);relay_put_be16(storage+6,120);storage[8]=floor;
 if(version==1){storage[8]=relay_crc8(7,storage,8);read_result=9;}
 else {storage[9]=relay_crc8(7,storage,9);read_result=10;}
}
int main(void){
 defaults();relay_settings_load();assert(relay_settings.tps_min==120 && relay_settings.max_trackers==16);
 for(unsigned version=1;version<=2;version++){
  defaults();saved(version,11,5,100,5);relay_settings_load();
  assert(relay_settings.tps_min==120 && relay_settings.max_trackers==16 && relay_settings.repair_floor==5);
  defaults();saved(version,16,5,100,5);relay_settings_load();
  assert(relay_settings.tps_min==100 && relay_settings.max_trackers==16);
  defaults();saved(version,12,4,120,4);relay_settings_load();
  assert(relay_settings.max_trackers==12 && relay_settings.max_repairs==4 && relay_settings.repair_floor==4);
 }
 defaults();saved(2,11,5,100,0);relay_settings_load();
 assert(relay_settings.max_trackers==11 && relay_settings.tps_min==100 && !relay_settings.repair_floor);
 defaults();saved(2,11,5,100,5);storage[9]^=1;relay_settings_load();assert(relay_settings.tps_min==120);
 defaults();assert(!esb_relay_layout_set("max_trackers",11));assert(!esb_relay_layout_set("max_repairs",3));
 assert(relay_settings.repair_floor==3);assert(!esb_relay_layout_set("max_repairs",5));assert(relay_settings.repair_floor==5);
 assert(!esb_relay_layout_set("repair_floor",1));assert(!esb_relay_layout_set("max_repairs",3));assert(relay_settings.repair_floor==1);
 assert(esb_relay_layout_set("max_repairs",0)==-EINVAL);assert(esb_relay_layout_set("repair_floor",4)==-EINVAL);
 unsigned before=writes;assert(!esb_relay_layout_set("max_repairs",3) && writes==before);
 assert(!esb_relay_layout_set("tps_min",100));assert(!esb_relay_layout_set("max_trackers",16));assert(relay_settings.max_repairs==3);
 assert(esb_relay_layout_set("max_trackers",0)==-EINVAL);assert(esb_relay_layout_set("max_trackers",256)==-EINVAL);
 assert(esb_relay_layout_set("tps_min",121)==-EINVAL);assert(esb_relay_layout_set("unknown",1)==-EINVAL);
 write_error=-EIO;before=relay_settings_generation;assert(esb_relay_layout_set("max_trackers",11)==-EIO);
 assert(relay_settings_generation==before && relay_settings.max_trackers==16);
 write_error=0;assert(esb_relay_layout_set("slot_ticks",15)==(CONFIG_RELAY_EXPERIMENTAL_SLOT_15?0:-EINVAL));
 for(unsigned target=100;target<=120;target+=20)for(unsigned n=1;n<=16;n++){
  struct relay_runtime_settings desired={16,5,16,target,120,5,0};
  struct relay_runtime_settings applied=relay_runtime_for_members(desired,n);
  unsigned r=(target==120?17:20)-n-1;r=r>5?5:r<1?1:r;
  assert(applied.max_repairs==r && applied.repair_floor==r && desired.max_repairs==5);
  relay_applied_settings=applied;for(unsigned f=0;f<24;f++)assert(relay_frame_repairs(f)==(r==1 && f%8==0?2:r));
 }
 defaults();assert(!esb_relay_layout_set("tps_min",100));assert(!esb_relay_layout_set("max_trackers",11));
 assert(!esb_relay_layout_set("second_b",2));assert(storage[0]==3 && storage[3]>>6==2);
 read_result=10;relay_settings.second_b=0;relay_settings_load();
 assert(relay_settings.second_b==2 && relay_settings.max_trackers==11 && relay_settings.tps_min==100);
 assert(esb_relay_layout_set("second_b",3)==-EINVAL);
 assert(!mutex && !depth);puts("Settings migration, exact defaults, explicit choices, write atomicity and per-member static R pass");
}
'''
with tempfile.TemporaryDirectory() as directory:
    p=Path(directory);(p/'settings.c').write_text(prefix+state+function('esb_relay_layout_print')+function('esb_relay_layout_set')+cases)
    for experimental in (0,1):
        subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined',f'-DCONFIG_RELAY_EXPERIMENTAL_SLOT_15={experimental}','-I'+str(HERE.parents[2]/'src/connection'),str(p/'settings.c'),'-o',str(p/'settings')],check=True)
        subprocess.run([str(p/'settings')],check=True)

# The reduced receiver must accept old 16-member NVS records by clamping the
# member ceiling, while keeping explicit repair/rate settings and wire layout.
capacity_cases = cases[:cases.index('int main(void)')] + r'''
int main(void) {
 defaults(); relay_settings_load(); assert(relay_settings.max_trackers == 10);
 for (unsigned version = 1; version <= 3; version++) {
  defaults(); saved(version,16,4,100,4); relay_settings_load();
  assert(relay_settings.max_trackers == 10 && relay_settings.max_repairs == 4);
  assert(relay_settings.tps_min == 100);
 }
 defaults(); saved(2,11,5,100,5); relay_settings_load();
 assert(relay_settings.max_trackers == 10 && relay_settings.tps_min == 120);
 defaults(); assert(esb_relay_layout_set("max_trackers",11) == -EINVAL);
 assert(!esb_relay_layout_set("max_trackers",10));
 assert(relay_settings.max_trackers == 10 && !mutex && !depth);
 puts("Ten-ID settings: saved ceilings clamped, migration and console bounds pass");
}
'''
with tempfile.TemporaryDirectory() as directory:
    p = Path(directory)
    (p/'settings.c').write_text(prefix + state + function('esb_relay_layout_print')
                               + function('esb_relay_layout_set') + capacity_cases)
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
        '-fsanitize=address,undefined', '-DCONFIG_RELAY_COORD_CAPACITY=10',
        '-DCONFIG_RELAY_EXPERIMENTAL_SLOT_15=0',
        '-I'+str(HERE.parents[2]/'src/connection'), str(p/'settings.c'),
        '-o', str(p/'settings')], check=True)
    subprocess.run([str(p/'settings')], check=True)
