#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tracker_events.h"
#include "connection/tracker_event_protocol.h"

uint32_t host_now;
static uint8_t records[256][16];
static uint32_t generations[256];
static uint32_t received_ms[256];
static size_t count;
static bool accept_output = true;
bool hid_write_tracker_event(const uint8_t record[16], uint32_t generation)
{
    if (!accept_output) return false;
    assert(count < 256);
    memcpy(records[count], record, 16);
    received_ms[count]=host_now;
    generations[count++] = generation;
    return true;
}
static void tick(uint32_t now)
{
    host_now = now;
    tracker_events_process(now);
}
static int control(uint8_t action, uint8_t target, uint8_t mask)
{
    uint8_t args[] = {1, action, target, mask};
    uint8_t result[12];
    return tracker_events_control(args, sizeof(args), tracker_events_usb_generation(), result);
}
static void subscribe(uint8_t target, uint8_t mask)
{
    assert(control(1, target, mask) == 0);
    tracker_events_process(host_now);
}
static struct tracker_event event(uint16_t seq, uint8_t type)
{
    return (struct tracker_event){.nonce=0x12345678, .event_seq=seq,
        .operation_id=0x2345, .tracker_id=2, .kind=CAL_KIND_IMU_ZRO,
        .event=type, .phase=CAL_PHASE_COLLECT,
        .outcome=type == CAL_EVENT_END ? CAL_OUTCOME_SUCCESS : CAL_OUTCOME_NONE};
}
static void receive(struct tracker_event e)
{
    uint8_t packet[17];
    assert(tracker_event_encode(packet, &e));
    assert(tracker_events_receive(packet, sizeof(packet), host_now));
    tracker_events_process(host_now);
}
static void clear_output(void) { count=0; }
static void dump(bool timestamps)
{
    for (size_t i=0; i<count; i++) {
        if(timestamps) printf("%u ",received_ms[i]);
        for (size_t j=0; j<16; j++) printf("%02x", records[i][j]);
        putchar('\n');
    }
    clear_output();
}

#include "esb_fixture.h"
static void golden(void)
{
    struct tracker_event e=event(0x3456, CAL_EVENT_BEGIN), decoded;
    uint8_t packet[17];
    const uint8_t expected[]={0xe0,2,1,2,0x78,0x56,0x34,0x12,0x56,0x34,0x45,0x23,1,4,0,0x43,0x45};
    const uint8_t hid[]={251,0,225,2,2,0x78,0x56,0x34,0x12,0x56,0x34,0x45,0x23,1,4,0};
    assert(tracker_event_encode(packet,&e));
    assert(memcmp(packet,expected,17)==0);
    assert(tracker_event_decode(packet,17,&decoded));
    assert(decoded.nonce==e.nonce && decoded.event_seq==e.event_seq);
    assert(tracker_event_decode_body(2,packet+2,15,&decoded));
    subscribe(255,31);
    receive(e); receive(e); receive(e);
    assert(count==1 && memcmp(records[0],hid,16)==0);
    assert(tracker_events_hid_valid(records[0],generations[0],host_now));
    dump(false);
}
static void decoder(void)
{
    struct tracker_event e=event(1,CAL_EVENT_BEGIN), decoded;
    uint8_t packet[32]={0};
    assert(tracker_event_encode(packet,&e));
    for (size_t n=0;n<=32;n++) {
        assert(tracker_event_decode(packet,n,&decoded)==(n==17));
        assert(tracker_event_decode_body(2,packet+2,n,&decoded)==(n==15));
        assert(tracker_events_receive(packet,n,0)==(n==17));
    }
    for (unsigned int a=0;a<256;a++) for(unsigned int b=0;b<256;b++) {
        packet[15]=a; packet[16]=b;
        assert(tracker_event_decode(packet,17,&decoded)==(a==0x43 && b==0x45));
    }
    packet[15]=0x43; packet[16]=0x45;
    packet[0]=0; assert(!tracker_events_receive(packet,17,0));
    packet[0]=0xe0; packet[2]=2; assert(!tracker_events_receive(packet,17,0));
    packet[2]=1; packet[1]=16; assert(!tracker_events_receive(packet,17,0));
}
static void timeout(void)
{
    subscribe(255,31);
    receive(event(1,CAL_EVENT_BEGIN)); clear_output();
    tick(9999); assert(count==0);
    tick(10000); assert(count==1 && records[0][2]==226 && records[0][3]==0x54);
    tick(10001); assert(count==1);
    receive(event(2,CAL_EVENT_END));
    assert(count==2 && records[1][2]==225 && records[1][3]==0x14);
}
static void storage(void)
{
    subscribe(255,31);
    receive(event(1,CAL_EVENT_BEGIN)); receive(event(2,CAL_EVENT_END));
    struct tracker_event e=event(3,CAL_EVENT_STEP); e.phase=CAL_PHASE_STORAGE;
    receive(e);
    e.event_seq=4; e.operation_id++; receive(e); /* STORAGE without a cached start. */
    clear_output(); tick(10000); assert(count==0);
}
static void nonce(void)
{
    subscribe(255,31);
    struct tracker_event old=event(1,CAL_EVENT_BEGIN), next=old;
    receive(old); clear_output();
    next.nonce++; receive(next);
    bool unknown=false,begin=false;
    for(size_t i=0;i<count;i++) {
        unknown |= records[i][2]==226 && records[i][3]==0x54;
        begin |= records[i][2]==225 && tracker_event_get32(records[i]+5)==next.nonce;
    }
    assert(unknown && begin); clear_output();
    old.event_seq=2; receive(old); assert(count==0);
    next.event_seq=2; receive(next); assert(count==1);
}
static void sequence(void)
{
    subscribe(255,31);
    receive(event(65534,CAL_EVENT_BEGIN));
    receive(event(65535,CAL_EVENT_STEP)); receive(event(0,CAL_EVENT_STEP));
    receive(event(65535,CAL_EVENT_STEP)); receive(event(65533,CAL_EVENT_STEP));
    assert(count==4); /* Unseen reordered history is observable, duplicates aren't. */
}
static void long_heartbeat(void)
{
    subscribe(255,31);
    struct tracker_event heartbeat=event(1,CAL_EVENT_BEGIN);
    receive(heartbeat); clear_output();
    struct tracker_event other=event(2,CAL_EVENT_NOTICE);
    other.operation_id=0; other.kind=TRACKER_EVENT_KIND_BUTTON;
    other.phase=BUTTON_CLICK_GROUP; other.detail=1;
    for(uint32_t seq=2;seq<=32770;seq++) {
        other.event_seq=(uint16_t)seq; receive(other); clear_output();
    }
    tick(9000); receive(heartbeat); assert(count==0);
    assert(control(2,255,31)==0);
    tick(10000); assert(count==0);
    tick(18999); assert(count==0);
    tick(19000); assert(count==1 && records[0][3]==0x54);
}
static void subscriptions(void)
{
    receive(event(1,CAL_EVENT_BEGIN)); assert(count==0);
    subscribe(2,1); clear_output();
    receive(event(2,CAL_EVENT_STEP)); assert(count==1);
    uint8_t saved[16]; memcpy(saved,records[0],16);
    uint32_t generation=generations[0];
    assert(control(2,3,1)<0);
    assert(control(2,2,1)==0);
    assert(tracker_events_hid_valid(saved,generation,0));
    subscribe(3,1); clear_output();
    assert(!tracker_events_hid_valid(saved,generation,0));
    receive(event(3,CAL_EVENT_STEP)); assert(count==0);
    subscribe(2,2); clear_output(); receive(event(4,CAL_EVENT_STEP)); assert(count==0);
    subscribe(2,1); clear_output(); receive(event(5,CAL_EVENT_STEP)); assert(count==1);
    generation=generations[0]; memcpy(saved,records[0],16);
    uint32_t usb=tracker_events_usb_generation();
    tracker_events_usb_reset(); assert(tracker_events_usb_generation()!=usb);
    assert(!tracker_events_hid_valid(saved,generation,0));
    clear_output(); receive(event(6,CAL_EVENT_STEP)); assert(count==0);
    uint8_t args[]={1,1,255,31}, result[12];
    assert(tracker_events_control(args,4,usb,result)<0);
    subscribe(255,31); clear_output(); tick(15000); clear_output();
    receive(event(7,CAL_EVENT_STEP)); assert(count==0);
    subscribe(255,31); receive(event(8,CAL_EVENT_STEP)); assert(count==1);
    generation=generations[0]; memcpy(saved,records[0],16);
    assert(control(3,255,0)==0);
    assert(!tracker_events_hid_valid(saved,generation,host_now));
    clear_output(); receive(event(9,CAL_EVENT_STEP)); assert(count==0);
    assert(control(2,255,31)<0);
}
static struct tracker_event state(uint16_t seq,uint8_t kind)
{
    struct tracker_event e=event(seq,CAL_EVENT_STATE);
    e.operation_id=0; e.kind=kind; e.phase=1;
    e.detail=kind==TRACKER_EVENT_KIND_FUSION_REST ? FUSION_BACKEND_VQF : TRACKER_REST_OBSERVED;
    return e;
}
static void rest(void)
{
    receive(state(1,TRACKER_EVENT_KIND_TRACKER_REST));
    receive(state(2,TRACKER_EVENT_KIND_FUSION_REST));
    subscribe(255,6); assert(count==2);
    for(size_t i=0;i<count;i++) assert(records[i][2]==226 && records[i][3]==CAL_EVENT_STATE);
    clear_output(); tick(10000); assert(control(2,255,6)==0);
    tick(14999); assert(count==0);
    tick(15000); assert(count==2);
    for(size_t i=0;i<count;i++) assert(records[i][2]==226 && records[i][14]==2);
    tick(15001); assert(count==2); clear_output();
    receive(state(3,TRACKER_EVENT_KIND_TRACKER_REST));
    receive(state(4,TRACKER_EVENT_KIND_FUSION_REST));
    assert(count==2 && records[0][2]==225 && records[1][2]==225);
    clear_output(); subscribe(255,6); assert(count==2);
}
static void independent(void)
{
    subscribe(255,31);
    receive(state(1,TRACKER_EVENT_KIND_TRACKER_REST));
    receive(state(2,TRACKER_EVENT_KIND_FUSION_REST));
    struct tracker_event notice=state(3,TRACKER_EVENT_KIND_POWER);
    notice.event=CAL_EVENT_NOTICE; notice.phase=POWER_WILL_WOM; notice.detail=POWER_WOM_NORMAL;
    receive(notice);
    notice.event_seq=4; notice.kind=TRACKER_EVENT_KIND_BUTTON; notice.phase=BUTTON_CLICK_GROUP; notice.detail=7;
    receive(notice); assert(count==4); clear_output();
    subscribe(255,31); assert(count==2);
    assert(records[0][13]!=records[1][13]);
    for(size_t i=0;i<count;i++) assert(records[i][13]==0x20 || records[i][13]==0x21);
}
static void pairing(void)
{
    subscribe(255,31);
    struct tracker_event e=event(1,CAL_EVENT_BEGIN);
    receive(e);
    uint8_t saved[16]; memcpy(saved,records[0],16);
    uint32_t generation=generations[0]; clear_output();
    uint8_t packet[17];
    for (unsigned t=0; t<16; ++t) {
        struct tracker_event old=e; old.tracker_id=t; old.event_seq=2;
        assert(tracker_event_encode(packet,&old));
        assert(tracker_events_receive(packet,17,host_now));
    }
    stored_trackers=16;
    tdma_active_mask=BIT(2);
    tdma_dynamic_active_count=1;
    tdma_dynamic_slot_ticks=TDMA_SLOT_TICKS_MAX_LEVEL0;
    tdma_config_epoch=1;
    tdma_config_packed[2]=tdma_pack_config(0, 1, tdma_dynamic_slot_ticks, tdma_config_epoch);
    atomic_set(&tdma_shadow_desired_mask, BIT(2));
    tdma_last_reconfig_time=host_now;
    tdma_last_reconfig_valid=true;
    esb_clear();
    assert(stored_trackers==0 && irq_depth==0);
    assert(tdma_active_mask==0 && tdma_dynamic_active_count==0);
    assert(tdma_last_reconfig_valid && tdma_last_reconfig_time==host_now);
    assert(!tracker_events_hid_valid(saved,generation,host_now));
    tracker_events_process(host_now); assert(count==0);
    e.nonce++; e.event_seq=1; receive(e); assert(count==1);
    clear_output();
    /* New RX may beat deferred cleanup after slot publication. It must
     * survive cleanup; old cached rest must never become a new snapshot. */
    tracker_events_pairing_invalidate(BIT(2));
    tracker_events_process(host_now); assert(count==0);
    e.nonce++; receive(e); assert(count==1); clear_output();
    tracker_events_pairing_cleanup();
    receive(e); assert(count==0);
}
static void rest_long_sequence(void)
{
    subscribe(255,31);
    receive(state(1,TRACKER_EVENT_KIND_TRACKER_REST)); clear_output();
    struct tracker_event notice=state(2,TRACKER_EVENT_KIND_BUTTON);
    notice.event=CAL_EVENT_NOTICE; notice.phase=BUTTON_CLICK_GROUP; notice.detail=1;
    for(uint32_t seq=2;seq<=32770;seq++) {
        notice.event_seq=(uint16_t)seq; receive(notice); clear_output();
    }
    struct tracker_event latest=state(32771,TRACKER_EVENT_KIND_TRACKER_REST);
    latest.phase=TRACKER_REST_NOT_REST;
    receive(latest); assert(count==1 && records[0][14]==TRACKER_REST_NOT_REST);
    clear_output(); subscribe(255,2);
    assert(count==1 && records[0][14]==TRACKER_REST_NOT_REST);
}
static void rest_delivery_retry(void)
{
    receive(state(1,TRACKER_EVENT_KIND_TRACKER_REST));
    accept_output=false; subscribe(255,2); assert(count==0);
    tracker_events_process(host_now); assert(count==0);
    accept_output=true; tracker_events_process(host_now);
    assert(count==1 && records[0][2]==226);
    tracker_events_process(host_now); assert(count==1);
}
static void json_rest(void)
{
    struct tracker_event e=state(1,TRACKER_EVENT_KIND_TRACKER_REST);
    receive(e); subscribe(255,2);
    tick(10000); assert(control(2,255,2)==0);
    tick(15000);
    receive(e); /* Same heartbeat restores freshness without a new transition. */
    subscribe(255,2);
    assert(count==4);
    dump(true);
}
static struct tracker_event power(uint16_t seq, uint8_t phase)
{
    struct tracker_event e=event(seq,CAL_EVENT_NOTICE);
    e.operation_id=0; e.kind=TRACKER_EVENT_KIND_POWER; e.phase=phase;
    return e;
}
static void assert_silence(uint8_t reason)
{
    assert(count==1 && records[0][2]==RCV_HID_OP_TRACKER_OBSERVATION);
    assert(records[0][3]==((CAL_OUTCOME_UNKNOWN<<4)|CAL_EVENT_END));
    assert(records[0][15]==reason);
}
static void power_timeout(void)
{
    const uint8_t phases[]={POWER_WILL_WOM,POWER_WILL_SHUTDOWN,POWER_WILL_REBOOT};
    subscribe(255,31);
    for(unsigned i=0;i<3;i++) {
        struct tracker_event e=event(1,CAL_EVENT_BEGIN); e.tracker_id=i; receive(e);
        e=power(2,phases[i]); e.tracker_id=i; receive(e);
    }
    receive(state(3,TRACKER_EVENT_KIND_TRACKER_REST));
    receive(state(4,TRACKER_EVENT_KIND_FUSION_REST));
    clear_output();
    subscribe(255,31); assert(count==2); /* Intent did not invalidate rest. */
    for(size_t i=0;i<count;i++) assert(records[i][3]==CAL_EVENT_STATE && records[i][14]==1);
    clear_output(); tick(9999); assert(count==0);
    tick(10000); assert(count==3);
    for(size_t i=0;i<count;i++) {
        assert(records[i][2]==226 && records[i][3]==0x54);
        assert(records[i][15]==(records[i][4]==2 ? CAL_REASON_RESET : CAL_REASON_POWER_DOWN));
    }
    clear_output(); assert(control(2,255,31)==0); tick(15000); assert(count==2);
    for(size_t i=0;i<count;i++) {
        assert(records[i][2]==226 && records[i][3]==0x56 && records[i][14]==2);
        assert(records[i][15]==(records[i][13]==TRACKER_EVENT_KIND_FUSION_REST
                               ? FUSION_BACKEND_VQF : TRACKER_REST_OBSERVED));
    }
}
static void power_cancel(void)
{
    subscribe(255,31);
    receive(event(65533,CAL_EVENT_BEGIN));
    receive(power(65534,POWER_WILL_WOM));
    receive(power(0,POWER_WOM_CANCELLED)); clear_output();
    receive(power(65535,POWER_WILL_WOM));
    /* Unseen history stays observable but cannot undo cancellation, including wrap. */
    assert(count==1 && records[0][2]==225 && records[0][14]==POWER_WILL_WOM);
    clear_output(); tick(10000); assert_silence(CAL_REASON_TRANSPORT_SILENCE);
}
static void power_reanchor(void)
{
    subscribe(255,31);
    struct tracker_event heartbeat=event(1,CAL_EVENT_BEGIN);
    receive(heartbeat); receive(power(4,POWER_WOM_CANCELLED)); clear_output();
    tick(9000); receive(heartbeat); assert(control(2,255,31)==0);
    tick(16000); receive(power(3,POWER_WILL_WOM));
    assert(count==1 && records[0][2]==225); /* Delivery window reanchored, not intent. */
    clear_output(); tick(19000); assert_silence(CAL_REASON_TRANSPORT_SILENCE);
}
static void power_expiry(void)
{
    subscribe(255,31);
    struct tracker_event heartbeat=event(1,CAL_EVENT_BEGIN);
    struct tracker_event notice=power(2,POWER_WILL_WOM);
    receive(heartbeat); receive(notice); clear_output();
    tick(9000); receive(notice); assert(count==0);
    /* Last activity at entry-window boundary; original 20s bound still wins. */
    host_now=10000; receive(heartbeat); assert(control(2,255,31)==0);
    tick(16000); receive(notice); clear_output();
    tick(19999); assert(count==0);
    tick(20000); assert_silence(CAL_REASON_TRANSPORT_SILENCE);
}
static void power_activity(void)
{
    subscribe(255,31);
    struct tracker_event heartbeat=event(1,CAL_EVENT_BEGIN);
    receive(heartbeat);
    struct tracker_event other=event(2,CAL_EVENT_BEGIN); other.operation_id++;
    receive(other); receive(power(3,POWER_WILL_WOM)); clear_output();
    host_now=1000; receive(other); clear_output();
    tick(9000); receive(heartbeat); assert(control(2,255,31)==0);
    host_now=10001; receive(heartbeat); /* Ongoing operation outlives expected entry. */
    assert(count==0); tick(11000); assert_silence(CAL_REASON_TRANSPORT_SILENCE);
}
static void power_new_operation(void)
{
    subscribe(255,31);
    receive(event(1,CAL_EVENT_BEGIN)); receive(power(2,POWER_WILL_WOM));
    struct tracker_event next=event(4,CAL_EVENT_BEGIN); next.operation_id++;
    receive(next);
    /* An unseen old warning arriving after the independent operation is not its cause. */
    receive(power(3,POWER_WILL_SHUTDOWN)); clear_output();
    tick(10000); assert(count==2);
    for(size_t i=0;i<count;i++) {
        assert(records[i][2]==226 && records[i][3]==0x54);
        assert(records[i][15]==(tracker_event_get16(records[i]+11)==next.operation_id
                               ? CAL_REASON_TRANSPORT_SILENCE : CAL_REASON_POWER_DOWN));
    }
}
static void power_boot(void)
{
    subscribe(255,31);
    receive(event(3,CAL_EVENT_BEGIN));
    receive(state(4,TRACKER_EVENT_KIND_TRACKER_REST));
    receive(state(5,TRACKER_EVENT_KIND_FUSION_REST));
    clear_output();
    receive(power(1,POWER_BOOT)); receive(power(2,POWER_WATCHDOG_RESET));
    assert(count==2); /* Delayed boot facts do not reset current-session state. */
    for(size_t i=0;i<count;i++) assert(records[i][2]==225 && records[i][3]==CAL_EVENT_NOTICE);
    clear_output(); subscribe(255,31); assert(count==2);
    clear_output(); tick(10000); assert_silence(CAL_REASON_TRANSPORT_SILENCE);
}
static void power_clear(void)
{
    const uint8_t phases[]={POWER_BOOT,POWER_WAKE,POWER_WATCHDOG_RESET};
    subscribe(255,31);
    for(unsigned i=0;i<3;i++) {
        struct tracker_event e=event(1,CAL_EVENT_BEGIN); e.tracker_id=i; receive(e);
        e=power(2,POWER_WILL_REBOOT); e.tracker_id=i; receive(e);
        e=power(3,phases[i]); e.tracker_id=i; receive(e);
    }
    clear_output(); tick(10000); assert(count==3);
    for(size_t i=0;i<count;i++) assert(records[i][2]==226 && records[i][3]==0x54
                                     && records[i][15]==CAL_REASON_TRANSPORT_SILENCE);
}
static void power_reset(void)
{
    subscribe(255,31);
    receive(event(1,CAL_EVENT_BEGIN)); receive(power(2,POWER_WILL_REBOOT));
    clear_output();
    struct tracker_event next=event(3,CAL_EVENT_BEGIN); next.nonce++; receive(next);
    assert(count==2 && records[0][2]==226 && records[0][15]==CAL_REASON_SESSION_CHANGED);
    clear_output(); tick(10000); assert_silence(CAL_REASON_TRANSPORT_SILENCE);
    struct tracker_event notice=power(4,POWER_WILL_WOM); notice.nonce=next.nonce;
    clear_output(); receive(notice); assert(count==1 && records[0][2]==225);
    tracker_events_pairing_invalidate(BIT(2)); tracker_events_pairing_cleanup();
    subscribe(255,31); clear_output();
    receive(event(5,CAL_EVENT_BEGIN)); clear_output();
    tick(20000); assert_silence(CAL_REASON_TRANSPORT_SILENCE);
}
static void json_power(void)
{
    subscribe(255,31);
    for(uint8_t phase=POWER_WILL_WOM;phase<=POWER_WATCHDOG_RESET;phase++) {
        struct tracker_event e=power(phase,phase);
        if(phase==POWER_WILL_WOM) e.detail=POWER_WOM_NORMAL;
        if(phase==POWER_WOM_CANCELLED) e.detail=POWER_WOM_FORCED;
        receive(e); receive(e); /* Only a single fact for each repeated notice. */
    }
    assert(count==7); dump(true);
    subscribe(255,31); assert(count==0); /* Lifecycle notices remain live-only. */
}
static void power_long_sequence(bool reanchor)
{
    subscribe(255,31);
    receive(power(1,POWER_WOM_CANCELLED)); clear_output();
    if(reanchor) {
        tick(16000); subscribe(255,31);
        receive(power(0,POWER_WILL_WOM)); clear_output();
    }
    struct tracker_event button=power(2,BUTTON_CLICK_GROUP);
    button.kind=TRACKER_EVENT_KIND_BUTTON; button.detail=1;
    for(uint32_t seq=2;seq<=32770;seq++) {
        button.event_seq=(uint16_t)seq; receive(button); clear_output();
    }
    receive(event(32771,CAL_EVENT_BEGIN));
    receive(power(32772,POWER_WILL_WOM)); clear_output();
    tick(host_now+10000); assert_silence(CAL_REASON_POWER_DOWN);
}
static void replay(void)
{
    char line[512]; subscribe(255,31); clear_output();
    bool explicit_clock=false;
    while(fgets(line,sizeof(line),stdin)) {
        if(line[0]=='#' || line[0]=='\n') continue;
        if(line[0]=='@') {
            explicit_clock=true;
            uint32_t target=(uint32_t)strtoul(line+1,NULL,10);
            assert(target>=host_now);
            while(target-host_now>5000) {
                tick(host_now+5000); dump(true); assert(control(2,255,31)==0);
            }
            tick(target); dump(true); assert(control(2,255,31)==0); continue;
        }
        if(line[0]=='!') {
            subscribe(255,31); dump(true); continue;
        }
        uint8_t packet[256]; size_t length=0; char *p=line;
        while(*p && *p!='\n' && *p!='#') {
            if(*p==' ' || *p=='\t' || *p=='\r') {p++; continue;}
            unsigned int value; int used;
            assert(sscanf(p,"%2x%n",&value,&used)==1 && used==2);
            assert(length<sizeof(packet)); packet[length++]=(uint8_t)value; p+=used;
        }
        assert(tracker_events_receive(packet,length,host_now));
        tracker_events_process(host_now); dump(true);
        if(!explicit_clock) host_now+=100;
        /* The replay host keeps its lease alive, like the production client. */
        assert(control(2,255,31)==0);
    }
}
int main(int argc,char **argv)
{
    assert(argc==2);
    if(!strcmp(argv[1],"golden")) golden();
    else if(!strcmp(argv[1],"decoder")) decoder();
    else if(!strcmp(argv[1],"timeout")) timeout();
    else if(!strcmp(argv[1],"storage")) storage();
    else if(!strcmp(argv[1],"nonce")) nonce();
    else if(!strcmp(argv[1],"sequence")) sequence();
    else if(!strcmp(argv[1],"long-heartbeat")) long_heartbeat();
    else if(!strcmp(argv[1],"subscriptions")) subscriptions();
    else if(!strcmp(argv[1],"rest")) rest();
    else if(!strcmp(argv[1],"independent")) independent();
    else if(!strcmp(argv[1],"replay")) replay();
    else if(!strcmp(argv[1],"pairing")) pairing();
    else if(!strcmp(argv[1],"composite")) composite();
    else if(!strcmp(argv[1],"rest-long-sequence")) rest_long_sequence();
    else if(!strcmp(argv[1],"rest-delivery-retry")) rest_delivery_retry();
    else if(!strcmp(argv[1],"json-rest")) json_rest();
    else if(!strcmp(argv[1],"power-timeout")) power_timeout();
    else if(!strcmp(argv[1],"power-cancel")) power_cancel();
    else if(!strcmp(argv[1],"power-reanchor")) power_reanchor();
    else if(!strcmp(argv[1],"power-expiry")) power_expiry();
    else if(!strcmp(argv[1],"power-activity")) power_activity();
    else if(!strcmp(argv[1],"power-new-operation")) power_new_operation();
    else if(!strcmp(argv[1],"power-boot")) power_boot();
    else if(!strcmp(argv[1],"power-clear")) power_clear();
    else if(!strcmp(argv[1],"power-reset")) power_reset();
    else if(!strcmp(argv[1],"json-power")) json_power();
    else if(!strcmp(argv[1],"power-long-sequence")) power_long_sequence(false);
    else if(!strcmp(argv[1],"power-long-reanchor")) power_long_sequence(true);
    else abort();
    return 0;
}
