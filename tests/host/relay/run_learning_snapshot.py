#!/usr/bin/env python3
"""Production publication keeps concurrent evidence and rejects a changed epoch."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay
from c_extract import extract_block

from pathlib import Path
import os, re, shlex, subprocess, tempfile
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
#define MAX_TRACKERS RC_MAX
static struct relay_coord relay_coord;
static uint32_t relay_session=42, relay_learn_lock_max;
static uint8_t relay_epoch=1;
static uint8_t relay_pairing_revision[MAX_TRACKERS];
static uint16_t tdma_active_mask=0x7ff;
static bool locked, change_epoch, change_session, change_members, change_pairing;
static unsigned irq_lock(void) { assert(!locked); locked=true; return 0; }
static void irq_unlock(unsigned key) { (void)key; assert(locked); locked=false; }
static uint32_t k_cycle_get_32(void) { return 1; }
void test_learn_tick(struct rc_learn_snapshot *snapshot, uint32_t now) {
 (void)now; assert(!locked);
 assert(snapshot->learn_members==tdma_active_mask);
 snapshot->choice[0][0].valid=true; snapshot->choice[0][0].helper=2;
 /* An ISR/report arrived after the snapshot. Publishing must retain it. */
 relay_coord.peer[2][0].heard=77;
 if(change_epoch) relay_epoch++;
 if(change_session) relay_session++;
 if(change_members) tdma_active_mask ^= 1;
 if(change_pairing) relay_pairing_revision[2]++;
}
'''+'\n#define relay_learn_snapshot_tick test_learn_tick\n'+src[a:b]+r'''
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
 change_epoch=false; change_session=true;
 relay_learn_update(2);
 assert(!relay_coord.choice[0][0].valid && !locked);
 change_session=false; change_members=true;
 relay_learn_update(2);
 assert(!relay_coord.choice[0][0].valid && !locked);
 change_members=false; change_pairing=true;
 relay_learn_update(2);
 assert(!relay_coord.choice[0][0].valid && !locked);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d); (p/'test.c').write_text(code)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-fsanitize=address,undefined','-Wall','-Wextra','-Werror','-I'+str(root/'src/connection'),str(p/'test.c'),str(root/'src/connection/relay_coord.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('Learning runs unlocked, preserves new peer evidence, rejects stale epoch publication')

# A proof update may invalidate both selected helpers between one-second TPS
# reports. The actual monitor block must refresh choices in that same pass.
monitor=src[src.index('\t\t/* Relay decisions every window'):src.index('\t\tfor (int i = 0; i < MAX_TRACKERS; i++)', src.index('\t\t/* Relay decisions every window'))]
policy=extract_block(src, r'^static void relay_auto_policy_update\([^;{}]*\)\s*\{')
protocol=(root/'src/connection/relay_protocol.h').read_text()
constants='\n'.join(
    re.search(r'^#define '+name+r'\b[^\n]*', text, re.M).group()
    for name, text in (
        ('TDMA_SHADOW_LEAVE_GRACE_MS', src),
        ('TDMA_RECONFIG_MIN_MS', src),
        ('TPS_MONITOR_INTERVAL_MS', src),
        ('RELAY_LAYOUT_ACTIVATION_MS', protocol),
    )
)
cadence=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "relay_coord.h"
''' + constants + r'''
static struct relay_coord relay_coord;
static struct { uint32_t planned_power_down_mask; } relay_member_diag;
static unsigned relay_profile=1, tdma_cap_level;
static uint32_t relay_query_protected_mask=10;
static bool relay_query_recovering=true, relay_query_suspending=true;
struct esb_auto_loss_snapshot { bool relay_eligible; };
#define esb_get_auto_loss_snapshot(e) ((e)->relay_eligible=true)
#define LOG_INF(...) ((void)0)
static enum relay_mode relay_user_mode = RELAY_MODE_AUTO;
static bool locked;
static unsigned tdma_active_mask=0x7ff, proof_updates, choice_updates, policy_updates, intent_reads, fence_calls;
static uint32_t event_mask=8;
static int64_t now_ms=500, relay_ota_hold_until;
static bool ota_active;
static bool esb_ota_relay_is_active(void){return ota_active;}
static bool esb_relay_ota_held_locked(int64_t now){return esb_ota_relay_is_active() || now < relay_ota_hold_until;}
static unsigned irq_lock(void){assert(!locked);locked=true;return 0;}
static void irq_unlock(unsigned key){assert(!key && locked);locked=false;}
static int64_t k_uptime_get(void){return now_ms;}
static uint32_t k_uptime_get_32(void){return (uint32_t)now_ms;}
static uint32_t tracker_events_power_down_mask(uint32_t now, uint32_t tail)
{
 assert(!locked && now==(uint32_t)now_ms && tail == TDMA_SHADOW_LEAVE_GRACE_MS + 2U * RC_LAYOUT_MAX_ACTIVATION_MS
        + TDMA_RECONFIG_MIN_MS + 2U * TPS_MONITOR_INTERVAL_MS);intent_reads++;return event_mask;
}
static bool relay_query_can_resume(void){assert(locked);return true;}
void relay_coord_auto_update(struct relay_coord *c, uint16_t members, uint32_t now, bool profile, bool eligible)
{
 assert(locked && c==&relay_coord && members==(tdma_active_mask & ~event_mask));
 assert(now==(uint32_t)now_ms && profile && eligible);policy_updates++;
}
#define IRQ_PROBE_BEGIN(name)
#define IRQ_PROBE_END(name)
#define relay_coord_set_enabled(c,e) ((c)->enabled=(e))
#define relay_coord_fence_native(c,t) do { assert(locked && (c)==&relay_coord && (t)==(uint32_t)now_ms); fence_calls++; } while (0)
#define relay_coord_tick(c,m,t) (proof_updates++)
static void relay_learn_update(uint32_t now){assert(!locked && now==(uint32_t)now_ms);choice_updates=proof_updates;}
''' + policy + r'''
static void monitor_pass(void){
''' + monitor + r'''
}
int main(void){
 monitor_pass();
 assert(proof_updates==1 && choice_updates==1 && policy_updates==1 && intent_reads==1 && fence_calls==1);
 assert(relay_member_diag.planned_power_down_mask==8 && relay_query_protected_mask==2);
 assert(relay_query_recovering && relay_query_suspending && tdma_active_mask==0x7ff);
 event_mask=0x7ff;monitor_pass();
 assert(proof_updates==2 && choice_updates==2 && policy_updates==2 && intent_reads==2 && fence_calls==1);
 assert(relay_member_diag.planned_power_down_mask==0x7ff && !relay_query_protected_mask);
 assert(!relay_query_recovering && relay_query_suspending && tdma_active_mask==0x7ff);
 /* Active OTA and post-OTA hold disable relay_coord and fence quality ticks. */
 ota_active=true;monitor_pass();
 assert(!relay_coord.enabled && proof_updates==2 && policy_updates==2 && fence_calls==2);
 ota_active=false;relay_ota_hold_until=now_ms+20000;monitor_pass();
 assert(!relay_coord.enabled && proof_updates==2 && policy_updates==2 && fence_calls==3);
 /* Once cooldown expires, relay_coord re-enables with a fresh native fence. */
 now_ms=relay_ota_hold_until;event_mask=8;monitor_pass();
 assert(relay_coord.enabled && proof_updates==3 && policy_updates==3 && fence_calls==4);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'cadence.c').write_text(cadence)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-fsanitize=address,undefined','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable','-I',str(root/'src/connection'),str(p/'cadence.c'),'-o',str(p/'cadence')],check=True)
 subprocess.run([str(p/'cadence')],check=True)
print('Every direct-proof update refreshes helper choices before TPS processing')
