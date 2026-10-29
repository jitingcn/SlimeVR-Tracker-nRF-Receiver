/* Hardware leaves only; parser and pairing-clear transaction are extracted verbatim. */
#include <zephyr/kernel.h>
#define MAX_TRACKERS 16
#define ESB_COMPOSITE_TYPE 0xfe
#define TEST_ALL_JOIN_CONFIG_GRACE_MS 1500
#define K_FOREVER 0
static void host_esb_log(const char *format, ...) { (void)format; }
#define LOG_ERR(...) host_esb_log(__VA_ARGS__)
#define LOG_DBG(...) host_esb_log(__VA_ARGS__)
#define LOG_WRN(...) host_esb_log(__VA_ARGS__)
#define LOG_INF(...) host_esb_log(__VA_ARGS__)
#define ATOMIC_INIT(x) (x)
#define K_SEM_DEFINE(name, initial, maximum) struct fixture_sem name = {initial}
typedef int atomic_t;
typedef int atomic_val_t;
struct fixture_sem { unsigned count; };
static int atomic_get(int *v) { return *v; }
static void atomic_and(int *v, int mask) { *v &= mask; }
static void atomic_or(int *v, int mask) { *v |= mask; }
static void atomic_set(int *v, int value) { *v=value; }
static void atomic_inc(int *v) { ++*v; }
static int test_all_generation, test_all_enabled;
static uint16_t test_all_requested_tps, test_all_effective_tps;
#include "esb_membership_declarations.inc"
static bool tracker_store_ready;
static int test_all_state_valid, test_all_confirmed_mask, test_all_ready_after_ms[16];
static uint8_t stored_trackers=3;
static uint64_t stored_tracker_addr[16];
static bool esb_clearing, esb_pairing;
static struct k_msgq esb_pairing_msgq;
static int tracker_store_lock;
static unsigned irq_depth;
static unsigned irq_lock(void) { return irq_depth++; }
static void irq_unlock(unsigned key) { irq_depth=key; }
static void k_sem_give(struct fixture_sem *sem) { assert(!irq_depth); sem->count=1; }
static void k_mutex_lock(int *lock, int timeout) { (void)lock; (void)timeout; }
static void k_mutex_unlock(int *lock) { (void)lock; }
static int tracker_roster_save(uint8_t count, const uint64_t *addresses) {
    assert(irq_depth==0 && count==0);
    for (unsigned i=0; i<MAX_TRACKERS; i++) assert(addresses[i]==0);
    return 0;
}
static void esb_ota_forget_tracker(uint8_t id) { assert(id<MAX_TRACKERS && irq_depth==0); }
static void data_collect_forget_tracker(uint8_t id) { assert(id<MAX_TRACKERS && irq_depth==0); }
static void test_all_invalidate(void) { assert(irq_depth!=0); }
static void raw_arq_reset(void) { assert(irq_depth!=0); }
static void tdma_shadow_forget_tracker(uint8_t id);
static void tracker_slot_reset(uint8_t id) {
    assert(irq_depth!=0 && id<MAX_TRACKERS);
    tdma_shadow_forget_tracker(id);
    tracker_events_pairing_invalidate(BIT(id));
}
static void checked_cleanup(void) {
    assert(irq_depth==0);
    tracker_events_pairing_cleanup();
}
static bool data_collect_is_active(void) { return false; }
static bool data_collect_batch_is_active(void) { return false; }
static void tdma_sync_stats_reset(void) { assert(!irq_depth); }
static void esb_auto_loss_invalidate(void) { assert(irq_depth); }
static void test_all_note_layout(uint32_t old, uint32_t next, int64_t at) { assert(!irq_depth); }
static void test_all_update_effective(uint8_t count, int64_t at, bool changed) { assert(!irq_depth); }
static struct { uint8_t data[72], length, rssi, pipe; } rx_payload;
static unsigned current_rx_ticks, sequence_calls, slot_calls, evidence_calls, pose_count;
static int sequence_result;
static uint8_t last_sequence, poses[8][16];
static struct packet_stats { uint8_t status_received, status_lost; } tracker_stats[16];
static int check_packet_sequence(uint8_t tracker, uint8_t sequence) {
    assert(tracker==2); sequence_calls++; last_sequence=sequence; return sequence_result;
}
static void tdma_check_slot(uint8_t tracker, unsigned ticks, uint8_t rssi) {
    (void)ticks; (void)rssi; assert(tracker==2); slot_calls++;
}
static void hid_write_packet_n(const uint8_t packet[16], uint8_t rssi) {
    (void)rssi; assert(pose_count<8); memcpy(poses[pose_count++],packet,16);
}
#define tracker_events_pairing_cleanup checked_cleanup
#include "esb_event_paths.inc"
#undef tracker_events_pairing_cleanup

static void composite(void) {
    /* Valid quat + info prefix, unknown tail, then the original pose seq. */
    rx_payload.pipe=1+(2%7);
    rx_payload.data[0]=0xfe; rx_payload.data[1]=2; rx_payload.data[2]=3;
    rx_payload.data[3]=1; memset(rx_payload.data+4,0x11,14);
    rx_payload.data[18]=0; memset(rx_payload.data+19,0x22,13);
    rx_payload.data[32]=0x99; rx_payload.data[33]=0xab;
    rx_payload.data[34]=42; rx_payload.length=35;
    composite_receive();
    assert(pose_count==2 && poses[0][0]==1 && poses[1][0]==0);
    assert(poses[0][2]==0x11 && poses[1][2]==0x22);
    assert(sequence_calls==1 && last_sequence==42 && slot_calls==1 && evidence_calls==1);
    assert((uint32_t)atomic_get(&tdma_shadow_pending_join_mask)==BIT(2));
    assert((uint32_t)atomic_get(&tdma_shadow_live_evidence_mask)==BIT(2));
    assert(tdma_membership_deadline_armed && tdma_membership_wake.count==1);
    sequence_result=4; composite_receive(); assert(pose_count==2 && sequence_calls==2);
    sequence_result=2; composite_receive(); assert(pose_count==2 && sequence_calls==3);
    sequence_result=0;
    /* Known truncated records and malformed E0 reject the whole frame before
     * pose delivery/accounting, rather than consuming the valid prefix. */
    rx_payload.data[32]=1; composite_receive();
    assert(pose_count==2 && sequence_calls==3);
    rx_payload.data[32]=TRACKER_EVENT_ESB_TYPE; composite_receive();
    assert(pose_count==2 && sequence_calls==3);
    struct tracker_event e=event(1,CAL_EVENT_BEGIN); uint8_t packet[17];
    assert(tracker_event_encode(packet,&e));
    memcpy(rx_payload.data+33,packet+2,15); rx_payload.data[48]=43; rx_payload.length=49;
    rx_payload.data[47]^=1; composite_receive();
    assert(pose_count==2 && sequence_calls==3);
    rx_payload.data[47]^=1; rx_payload.data[2]=4; composite_receive();
    assert(pose_count==2 && sequence_calls==3);
    rx_payload.data[2]=3; composite_receive(); tracker_events_process(host_now);
    assert(pose_count==4 && sequence_calls==4 && last_sequence==43);
    unsigned observed=evidence_calls;
    uint32_t last_seen=(uint32_t)atomic_get(&tdma_shadow_last_seen_ms[2]);
    host_now++;
    rx_payload.pipe=1; /* Wrong pipe still carries poses, but is not admission evidence. */
    composite_receive();
    assert(pose_count==6 && sequence_calls==5 && evidence_calls==observed);
    assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[2])==last_seen);
}
