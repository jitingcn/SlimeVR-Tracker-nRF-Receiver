#!/usr/bin/env python3
"""Production publication keeps concurrent evidence and rejects a changed epoch."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

from pathlib import Path
import os, shlex, subprocess, tempfile
root=Path(__file__).resolve().parents[3]
src=select_relay((root/'src/connection/esb.c').read_text())
a=src.index('static void relay_learn_update(uint32_t now)\n{')
b=src.index('\nstatic void esb_stats_thread(void)\n{',a)
code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "relay_coord.h"
#define MAX(a,b) ((a)>(b)?(a):(b))
static struct relay_coord relay_coord;
static uint32_t relay_session=42, relay_learn_lock_max;
static uint8_t relay_epoch=1;
static uint16_t tdma_active_mask=0x7ff;
static bool locked, change_epoch;
static unsigned irq_lock(void) { assert(!locked); locked=true; return 0; }
static void irq_unlock(unsigned key) { (void)key; assert(locked); locked=false; }
static uint32_t k_cycle_get_32(void) { return 1; }
void test_learn_tick(struct relay_coord *snapshot, uint32_t now) {
 (void)now; assert(!locked);
 assert(snapshot->learn_members==tdma_active_mask);
 snapshot->choice[0][0].valid=true; snapshot->choice[0][0].helper=2;
 /* An ISR/report arrived after the snapshot. Publishing must retain it. */
 relay_coord.peer[2][0].heard=77;
 if(change_epoch) relay_epoch++;
}
'''+'\n#define relay_coord_learn_tick test_learn_tick\n'+src[a:b]+r'''
int main(void) {
 relay_coord.peer[2][0]=(struct rc_peer){.expected=30,.evidence_ms=0};
 relay_coord.peer[3][0]=(struct rc_peer){.expected=30,.heard=29,.evidence_ms=10001};
 relay_learn_update(10000);
 assert(relay_coord.peer[2][0].expected==0);
 assert(relay_coord.peer[3][0].expected==30);
 assert(relay_coord.choice[0][0].valid && relay_coord.choice[0][0].helper==2);
 assert(relay_coord.peer[2][0].heard==77 && !locked);
 relay_coord.choice[0][0].valid=false; change_epoch=true;
 relay_learn_update(2);
 assert(!relay_coord.choice[0][0].valid && !locked);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d); (p/'test.c').write_text(code)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-I'+str(root/'src/connection'),str(p/'test.c'),str(root/'src/connection/relay_coord.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('Learning runs unlocked, preserves new peer evidence, rejects stale epoch publication')

# A proof update may invalidate both selected helpers between one-second TPS
# reports. The actual monitor block must refresh choices in that same pass.
monitor=src[src.index('\t\t/* Relay decisions every window'):src.index('\t\tfor (int i = 0; i < MAX_TRACKERS; i++)', src.index('\t\t/* Relay decisions every window'))]
cadence=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
static struct { bool enabled; } relay_coord;
static bool relay_user_enabled=true;
static unsigned tdma_active_mask=0x7ff, proof_updates, choice_updates;
static bool esb_ota_relay_is_active(void){return false;}
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned key){(void)key;}
static uint32_t k_uptime_get_32(void){return 500;}
#define IRQ_PROBE_BEGIN(name)
#define IRQ_PROBE_END(name)
#define relay_coord_set_enabled(c,e) ((c)->enabled=(e))
#define relay_coord_tick(c,m,t) (proof_updates++)
static void relay_learn_update(uint32_t now){assert(now==500);choice_updates=proof_updates;}
static void monitor_pass(void){
'''+monitor+r'''
}
int main(void){monitor_pass();assert(proof_updates==1 && choice_updates==1);return 0;}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'cadence.c').write_text(cadence)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable',str(p/'cadence.c'),'-o',str(p/'cadence')],check=True)
 subprocess.run([str(p/'cadence')],check=True)
print('Every direct-proof update refreshes helper choices before TPS processing')
