"""Run actual unlocked B planning and fence its publication context."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

from pathlib import Path
import os,shlex,subprocess,tempfile
root=Path(__file__).resolve().parents[3]
src=select_relay((root/'src/connection/esb.c').read_text());a=src.index('static void relay_broadcast_plan(');b=src.index('/* B is prepared outside the radio ISR.',a)
code=r'''
#define IRQ_PROBE_BEGIN(name)
#define IRQ_PROBE_END(name)
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "relay_coord.h"
struct esb_payload {uint8_t data[72];};
static struct relay_coord relay_coord;
static struct relay_runtime_settings relay_applied_settings={16,RELAY_V3_REPAIR_SLOTS,16,100,120,RELAY_V3_REPAIR_SLOTS,0};
static bool esb_initialized=true,esb_pairing,locked;
static uint8_t relay_profile=1,relay_epoch=2;
static uint16_t tdma_active_mask=0x7ff;
static uint32_t relay_session=42,relay_origin=100;
static unsigned locks, plans;
static unsigned irq_lock(void){assert(!locked);locked=true;locks++;return 0;}
static void irq_unlock(unsigned key){(void)key;assert(locked);locked=false;}
static uint32_t k_uptime_get_32(void){assert(!locked);return 5000;}
static void plan_watch(struct rc_plan_snapshot *snapshot,struct relay_control *c){
 assert(!locked);assert(snapshot->learn.t[15].direct_permille==987);
 assert(snapshot->learn.peer[15][14].heard==77);
 assert(snapshot->watch[0].frame==12 && snapshot->receipt[0].received==8);
 relay_plan_snapshot_watch(snapshot,c);plans++;
 /* Concurrent evidence must survive; planning publishes no coordinator state. */
 relay_coord.peer[15][14].heard=88;
}
static void plan_grants(struct rc_plan_snapshot *snapshot,struct relay_control *c,struct rc_grant_plan *p,uint32_t now){assert(!locked);relay_plan_snapshot_grants(snapshot,c,p,now);plans++;}
static void encode(uint8_t *wire,const struct relay_control *c){assert(!locked);relay_control_encode(wire,c);plans++;}
#define relay_plan_snapshot_watch plan_watch
#define relay_plan_snapshot_grants plan_grants
#define relay_control_encode encode
'''+src[a:b].replace('#endif', '')+r'''
int main(void){
 relay_coord_init(&relay_coord);relay_coord.v3_requested=true;
 for(unsigned i=0;i<11;i++)relay_coord_note_caps(&relay_coord,i,RELAY_INFO_CAPS);
 relay_coord.t[15].direct_permille=987;relay_coord.peer[15][14].heard=77;
 relay_coord.watch[0].frame=12;relay_coord.receipt[0].received=8;
 struct relay_control c={.profile=1,.session=42,.epoch=2,.members=0x7ff,.expected=0x7ff,.repair_count=RELAY_V3_REPAIR_SLOTS};
 struct rc_grant_plan p;struct esb_payload wire;
 relay_broadcast_plan(&c,&p,&wire);
 assert(plans==3 && !locked && locks==RC_MAX+RELAY_CACHE_MAX_FRAMES+1);
 assert(relay_coord.peer[15][14].heard==88);
 assert(relay_broadcast_context_current(&c,100));
 c.epoch++;assert(!relay_broadcast_context_current(&c,100));c.epoch--;
 c.session++;assert(!relay_broadcast_context_current(&c,100));c.session--;
 c.members^=1;assert(!relay_broadcast_context_current(&c,100));c.members^=1;
 assert(!relay_broadcast_context_current(&c,101));
 esb_pairing=true;assert(!relay_broadcast_context_current(&c,100));esb_pairing=false;
 relay_coord.v3_requested=false;assert(!relay_broadcast_context_current(&c,100));
 return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.c').write_text(code)
 for index in (0,1):
  subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-fsanitize=address,undefined','-Wall','-Wextra','-Werror','-Wno-unknown-attributes','-DRELAY_HOST_REPAIR_SLOTS='+str(3 if index else 5),'-I'+str(root/'src/connection'),str(p/'test.c'),str(root/'src/connection/relay_coord.c'),'-o',str(p/'test')],check=True)
  subprocess.run([str(p/'test')],check=True)
print('B planning runs unlocked, retains concurrent evidence and fences all network context changes')

assert "static struct rc_grant_plan plan;" in (root / "src/connection/esb.c").read_text()
