"""Exercise the production deferred queues, including overload ordering."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

from pathlib import Path
import os, subprocess, tempfile, sys, shlex
here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent / 'harness'))
from c_extract import extract_block
source = select_relay((here.parents[2] / 'src/connection/esb.c').read_text())
code = source[source.index('struct relay_learning_record {'):source.index('struct composite_result {')]
# Thread loops are exercised through their queue operations below.
for name in ('relay_learning_thread', 'relay_hid_thread'):
    code = code.replace(extract_block(code, rf'^static void {name}\(void\)\s*\{{'), '')
prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "relay_coord.h"
#define ARG_UNUSED(x) (void)(x)
#define K_NO_WAIT 0
#define K_THREAD_DEFINE(...)
struct queue { unsigned size, capacity, head, count; unsigned char *data; };
#define K_MSGQ_DEFINE(n,s,c,a) static unsigned char n##_data[(s)*(c)]; static struct queue n = {s,c,0,0,n##_data}
static int k_msgq_put(struct queue *q, const void *p, int wait) {
 (void)wait; if(q->count == q->capacity) return -1;
 memcpy(q->data+((q->head+q->count)%q->capacity)*q->size,p,q->size);q->count++;return 0;
}
static int k_msgq_get(struct queue *q, void *p, int wait) {
 (void)wait;if(!q->count)return -1;memcpy(p,q->data+q->head*q->size,q->size);
 q->head=(q->head+1)%q->capacity;q->count--;return 0;
}
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned key){(void)key;}
static struct relay_coord relay_coord;
static uint32_t relay_session=42;
static uint8_t relay_epoch=3;
static unsigned hid_count;
static bool stalled;
static bool hid_try_write_packet_n(const uint8_t *data,uint8_t rssi){if(stalled)return false;assert(data[0]==hid_count++);assert(rssi==50);return true;}
'''
cases = r'''
int main(void) {
 relay_coord_init(&relay_coord);
 for(unsigned i=0;i<129;i++){uint8_t p[16]={i};relay_hid_packet(p,50);p[0]=0;}
 assert(hid_count==0 && relay_hid_overflow==1 && relay_hid_dropped_output==1 && relay_hid_queue.count==128);
 struct relay_hid_record output;bool pending=false;stalled=true;
 assert(!relay_hid_step(&output,&pending)&&pending&&output.data[0]==0);
 assert(!relay_hid_step(&output,&pending)&&pending&&relay_hid_queue.count==127);
 uint8_t newer[16]={128};relay_hid_packet(newer,50);
 stalled=false;
 for(unsigned i=0;i<129;i++)assert(relay_hid_step(&output,&pending));
 assert(hid_count==129 && relay_hid_enqueued==129 && relay_hid_refused==2 && !pending);
 assert(!relay_hid_step(&output,&pending));
 for(unsigned i=0;i<17;i++)relay_defer_capture(&relay_coord,1,2,50,100+i);
 assert(relay_learning_overflow==1 && relay_coord.peer[1][2].evidence_ms==116);
 struct relay_control c={.session=42,.epoch=3,.members=2,.repair_count=5};
 c.grants[0].owner=1;
 struct relay_peer_report p={.session=42,.epoch=3,.helper=1,.count=1};
 p.entries[0]=(struct relay_peer_observation){.child=2,.expected=10,.heard=7,.rssi=50};
 relay_defer_report(&relay_coord,&c,0,&p,200);
 c.session=99;p.entries[0].heard=0;
 struct relay_learning_record r;
 assert(!k_msgq_get(&relay_learning_queue,&r,0));relay_learning_apply(&r);
 assert(relay_coord.peer[1][2].heard==7 && relay_coord.peer[1][2].expected==10);
 relay_epoch++;relay_learning_apply(&r);assert(relay_coord.peer[1][2].heard==7);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    c=Path(tmp)/'deferred.c';exe=Path(tmp)/'deferred';c.write_text(prefix+code+cases)
    subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(here.parents[2]/'src/connection'),str(c),str(here.parents[2]/'src/connection/relay_coord.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('Deferred HID and learning copies, overflow order and epoch fencing passed')
