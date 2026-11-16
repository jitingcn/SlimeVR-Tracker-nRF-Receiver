/* Drive production quality accounting and profile policy with eleven members. */
#include "relay_coord.h"
#include <assert.h>
#include <stdio.h>
#define MEMBERS 0x7ff
static struct relay_coord rc;
static uint32_t now;
static uint16_t roster;
static void setup(void) {
    relay_coord_init(&rc); rc.enabled = true; now = 100; roster = MEMBERS;
    for (unsigned i=0;i<11;i++) relay_coord_note_caps(&rc,i,
        (RELAY_PROTOCOL_VERSION << RELAY_CAP_VERSION_SHIFT) | RELAY_CAP_HELPER);
}
/* mode 0 good, 1 sustained 40% loss on one member, 2 repaired, 3 outage,
 * 10 outage with every frame delivered by relay. */
static void step(unsigned mode, bool applied, bool eligible) {
    now += RC_WINDOW_MS;
    for (unsigned i=0;i<11;i++) {
        if (i == 3 && mode == 10) {
            for (unsigned n=0;n<50;n++)
                relay_coord_note_frame(&rc,i,RC_RELAYED,RC_SEQ_NORMAL,0,70,now);
            continue;
        }
        if (i == 3 && mode == 3) continue;
        unsigned direct = i == 3 && mode ? (mode == 4 ? 45 : mode == 5 ? 5 : 30) : 50;
        unsigned gaps = i == 3 ? (mode == 1 ? 20 : mode == 4 ? 5 : 0) : 0;
        if (mode >= 6) {
            direct = i == 3 ? (mode == 7 || mode == 9 ? 85 : 90) : 85 + i % 9;
            if (mode == 8 && i == 4) direct = 60;
            gaps = 100 - direct;
        }
        for (unsigned n=0;n<direct;n++)
            relay_coord_note_frame(&rc,i,RC_DIRECT,RC_SEQ_NORMAL,
                n == 0 ? gaps : 0,70,now);
        if (i == 3 && mode == 2)
            for (unsigned n=0;n<20;n++)
                relay_coord_note_frame(&rc,i,RC_RELAYED,RC_SEQ_NORMAL,0,70,now);
    }
    relay_coord_tick(&rc,roster,now);
    relay_coord_auto_update(&rc,roster,now,applied,eligible);
}
static void run(unsigned ticks,unsigned mode,bool applied) {
    while(ticks--) step(mode,applied,true);
}
/* Applied ticks until relay exits, or limit + 1. */
static unsigned until_exit(unsigned mode,unsigned limit) {
    unsigned n = 0;
    while (rc.auto_engaged && n <= limit) { step(mode,true,true); n++; }
    return n;
}
/* One window of `expected` frames per member, `direct` of them direct. */
static void step_frames(const unsigned direct[11],unsigned expected,bool applied) {
    now += RC_WINDOW_MS;
    for (unsigned i=0;i<11;i++)
        for (unsigned n=0;n<direct[i];n++)
            relay_coord_note_frame(&rc,i,RC_DIRECT,RC_SEQ_NORMAL,
                n == 0 ? expected - direct[i] : 0,70,now);
    relay_coord_tick(&rc,roster,now);
    relay_coord_auto_update(&rc,roster,now,applied,true);
}
/* A full recovery window is 40 judgements after the first clean one. */
#define WINDOW_TICKS (RC_AUTO_RECOVER_MS / RC_WINDOW_MS)
/* Weak judgements until a member's weak streak (from its second judgement)
 * has lasted RC_AUTO_ENTER_MS; the now | 1 start marker adds one more. */
#define SUSTAIN_TICKS (RC_WEAK_WINDOWS + RC_AUTO_ENTER_MS / RC_WINDOW_MS)
/* A member whose 10 s weak streak completes on the tick the roster changes
 * still owes recovery to 900 per mille, even if its next judgement is 890. */
static void test_roster_change_keeps_poor_evidence(void) {
    setup(); run(24,1,false); assert(rc.auto_engaged);
    run(10,6,true);
    run(SUSTAIN_TICKS,8,true);
    assert(!(rc.auto_poor_members & (1U << 4))); /* streak one tick short */
    roster = MEMBERS & ~(1U << 10); step(8,true,true);
    assert(rc.t[4].direct_permille == 600);
    assert(rc.auto_poor_members & (1U << 4));
    run(80,6,true); assert(rc.auto_engaged); /* member 4 only at 89% */
    /* The window in progress may already pool >= 900 once it reaches 1000. */
    assert(until_exit(0,3 * WINDOW_TICKS) <= 2 * WINDOW_TICKS + 2);
}
/* (a) Hardware run 2: untouched tracker 0 at 770-960 per mille, never two
 * judgements in a row below 800; the trigger member back at 980. */
static void test_noisy_untouched_member_allows_exit(void) {
    static const unsigned noisy[] = {46,58,51,47,54,53,47,56,49,52}; /* of 60 */
    unsigned d[11];
    setup(); run(24,1,false); assert(rc.auto_engaged);
    assert(rc.auto_poor_members == (1U << 3));
    run(70,2,true); assert(rc.auto_engaged); /* hold met, member 3 still weak */
    unsigned n;
    for (n = 0; rc.auto_engaged && n < 400; n++) {
        for (unsigned i=0;i<11;i++) d[i] = 57;
        d[0] = noisy[n % 10];
        d[3] = 59;
        step_frames(d,60,true);
        assert(rc.t[0].weak_windows < RC_WEAK_WINDOWS);
    }
    assert(rc.auto_poor_members == 0);
    /* The first clean judgement opens the window; the now | 1 start
     * marker can add one tick. */
    assert(n >= WINDOW_TICKS + 1 && n <= WINDOW_TICKS + 2);
    /* Hold still applies: same noise right after application. */
    setup(); run(24,1,false); assert(rc.auto_engaged);
    for (n = 0; rc.auto_engaged && n < 400; n++) {
        for (unsigned i=0;i<11;i++) d[i] = 57;
        d[0] = noisy[n % 10];
        d[3] = 59;
        step_frames(d,60,true);
    }
    assert(n >= RC_AUTO_HOLD_MS / RC_WINDOW_MS && n <= RC_AUTO_HOLD_MS / RC_WINDOW_MS + 2);
}
/* (b) The trigger member pooled at 880 never exits, even with half of its
 * judgements at 950; pooled 920 exits although half of them are 880. */
static void test_trigger_member_pooled(void) {
    unsigned d[11];
    setup(); run(24,1,false); assert(rc.auto_engaged);
    for (unsigned n = 0; n < 400; n++) {
        for (unsigned i=0;i<11;i++) d[i] = 100;
        d[3] = n & 1 ? 81 : 95; /* of 100: 810 / 950, pooled 880 */
        step_frames(d,100,true);
    }
    assert(rc.auto_engaged && rc.auto_poor_members == (1U << 3));
    unsigned n;
    for (n = 0; rc.auto_engaged && n < 400; n++) {
        for (unsigned i=0;i<11;i++) d[i] = 100;
        d[3] = n & 1 ? 88 : 96; /* pooled 920 */
        step_frames(d,100,true);
    }
    assert(!rc.auto_engaged && n > WINDOW_TICKS && n <= 2 * WINDOW_TICKS + 2);
}
/* (c) An untouched member weak continuously for the 10 s entry window during
 * relay is remembered and owes pooled 900; it stops owing once it leaves, and
 * rejoins unremembered. */
static void test_new_weak_member(void) {
    setup(); run(24,1,false); run(10,0,true);
    step(8,true,true); assert(!(rc.auto_poor_members & (1U << 4)));
    step(8,true,true); assert(!(rc.auto_poor_members & (1U << 4)));
    assert(!rc.auto_good_since); /* the streak still restarts recovery */
    unsigned weak;
    for (weak = 2; !(rc.auto_poor_members & (1U << 4)) && weak < 100; weak++) step(8,true,true);
    /* Remembered once weak (from its second judgement) for 10 s. */
    assert(weak >= SUSTAIN_TICKS && weak <= SUSTAIN_TICKS + 1);
    run(200,6,true); assert(rc.auto_engaged); /* member 4 pooled 890 */
    /* The window in progress may already pool >= 900 once it reaches 1000. */
    assert(until_exit(0,3 * WINDOW_TICKS) <= 2 * WINDOW_TICKS + 2);
    setup(); run(24,1,false); run(60,2,true); run(SUSTAIN_TICKS + 1,8,true); /* hold met */
    assert(rc.auto_poor_members == ((1U << 3) | (1U << 4)));
    roster = MEMBERS & ~(1U << 4); step(6,true,true);
    roster = MEMBERS; step(6,true,true);
    assert(!(rc.auto_poor_members & (1U << 4)));
    unsigned n = until_exit(6,3 * WINDOW_TICKS); /* member 4 at 89%, untouched now */
    assert(n >= WINDOW_TICKS + 1 && n <= WINDOW_TICKS + 2);
}
/* M1 (reviewer): two judgements at 46/60, then a long 53/60 (883) on an
 * untouched member. The streak restarts recovery but is not remembered, so
 * relay exits after one clean window. Old rule: owed 900 forever. */
static void test_brief_streak_not_remembered(void) {
    unsigned d[11];
    setup(); run(24,1,false); assert(rc.auto_engaged);
    run(70,2,true); assert(rc.auto_engaged); /* hold met */
    for (unsigned i=0;i<11;i++) d[i] = 57;
    d[3] = 59;
    for (unsigned n=0;n<10;n++) step_frames(d,60,true);
    assert(rc.auto_engaged && rc.auto_good_since);
    d[0] = 46; step_frames(d,60,true); step_frames(d,60,true);
    assert(rc.t[0].weak_windows == RC_WEAK_WINDOWS && !rc.auto_good_since);
    assert(!(rc.auto_poor_members & 1U));
    d[0] = 53;
    unsigned n;
    for (n = 0; rc.auto_engaged && n < 1200; n++) step_frames(d,60,true); /* 600 s */
    assert(!rc.auto_engaged && n >= WINDOW_TICKS + 1 && n <= 2 * WINDOW_TICKS + 2);
    /* Sustained: the same member weak for 10 s now owes 900 and holds relay
     * at 53/60 until it recovers. One tick short of 10 s is not enough. */
    setup(); run(24,1,false); run(70,2,true);
    d[0] = 46;
    for (n = 0; n < SUSTAIN_TICKS - 1; n++) step_frames(d,60,true);
    assert(!(rc.auto_poor_members & 1U));
    d[0] = 53; step_frames(d,60,true);
    for (n = 0; n < 400; n++) step_frames(d,60,true);
    assert(!rc.auto_engaged); /* 9.5 s weak: untouched */
    setup(); run(24,1,false); run(70,2,true);
    d[0] = 46;
    for (n = 0; n < SUSTAIN_TICKS + 1; n++) step_frames(d,60,true);
    assert(rc.auto_poor_members & 1U);
    d[0] = 53;
    for (n = 0; n < 400; n++) step_frames(d,60,true);
    assert(rc.auto_engaged); /* remembered, pooled 883 */
    d[0] = 59;
    for (n = 0; rc.auto_engaged && n < 400; n++) step_frames(d,60,true);
    assert(!rc.auto_engaged && n <= 2 * WINDOW_TICKS + 2);
}
/* M2 (reviewer): forgetting an ID between policy ticks (re-pair into the same
 * slot) must drop the old session's remembered bit and balance. The new
 * session at 53/60 is untouched, so relay exits after one clean window. */
static void test_forget_rejoin(void) {
    unsigned d[11];
    setup(); run(24,1,false); assert(rc.auto_poor_members == (1U << 3));
    run(70,2,true); assert(rc.auto_engaged); /* hold met, member 3 still at 600 */
    relay_coord_forget(&rc,3,now);
    relay_coord_note_caps(&rc,3,(RELAY_PROTOCOL_VERSION << RELAY_CAP_VERSION_SHIFT) | RELAY_CAP_HELPER);
    assert(!(rc.auto_poor_members & (1U << 3)) && !rc.auto_balance[3] && !rc.auto_good_since);
    for (unsigned i=0;i<11;i++) d[i] = 57;
    d[3] = 53;
    unsigned n;
    for (n = 0; rc.auto_engaged && n < 400; n++) step_frames(d,60,true);
    assert(!rc.auto_engaged && n <= WINDOW_TICKS + 3);
}
int main(void) {
    setup(); run(400,0,false); assert(!rc.auto_engaged);
    run(400,4,false); assert(!rc.auto_engaged); /* 10% loss stays direct */
    setup(); run(400,9,false); assert(!rc.auto_engaged); /* 15% loss stays direct */
    for(unsigned n=0;n<20;n++) {
        run(12,1,false); assert(!rc.auto_engaged);
        run(8,0,false); assert(!rc.auto_engaged);
    }
    setup(); run(20,1,false); assert(!rc.auto_engaged);
    run(4,1,false); assert(rc.auto_engaged);
    run(200,2,true); assert(rc.auto_engaged); /* perfect delivery, poor direct */
    assert(rc.t[3].direct_permille == 600);
    run(38,0,true); assert(rc.auto_engaged);
    run(4,0,true); assert(!rc.auto_engaged);
    setup(); run(4,0,false); run(10,3,false); assert(!rc.auto_engaged);
    run(3,3,false); assert(rc.auto_engaged);
    /* Direct cut, relay delivering every frame: delivery never masks the
     * silent direct path, so relay holds until the direct link returns. */
    setup(); run(4,0,false); run(13,3,false); assert(rc.auto_engaged);
    run(400,10,true); assert(rc.auto_engaged && (rc.auto_poor_members & (1U << 3)));
    assert(until_exit(0,3 * WINDOW_TICKS) <= WINDOW_TICKS + 2);
    setup(); run(12,5,false); run(13,3,false); assert(!rc.auto_engaged);
    run(2,3,false); assert(rc.auto_engaged); /* idle timeout leaves staging margin */
    setup(); run(24,1,false); assert(rc.auto_engaged);
    run(40,0,true); assert(rc.auto_engaged); /* hold measured from application */
    run(22,0,true); assert(!rc.auto_engaged);
    setup(); run(18,1,false); step(1,false,false);
    run(18,1,false); assert(!rc.auto_engaged); /* excluded mode breaks window */
    run(5,1,false); assert(rc.auto_engaged);
    rc.enabled = false; step(1,true,true); assert(!rc.auto_engaged);
    /* Hardware fleet: untouched members at 85-93%, triggering member 90%.
     * 85% on a previously poor member must still refuse recovery. */
    setup(); run(24,1,false); assert(rc.auto_engaged);
    run(80,7,true); assert(rc.auto_engaged);
    unsigned n = until_exit(6,3 * WINDOW_TICKS);
    assert(n > WINDOW_TICKS && n <= 2 * WINDOW_TICKS + 2);
    /* A different member poor for 10 s during relay also owes recovery. */
    setup(); run(24,1,false); run(80,8,true);
    assert(rc.auto_engaged && (rc.auto_poor_members & (1U << 4)));
    run(80,6,true); assert(rc.auto_engaged); /* member 4 only at 89% */
    /* The window in progress may already pool >= 900 once it reaches 1000. */
    assert(until_exit(0,3 * WINDOW_TICKS) <= 2 * WINDOW_TICKS + 2);
    test_roster_change_keeps_poor_evidence();
    test_noisy_untouched_member_allows_exit();
    test_trigger_member_pooled();
    test_new_weak_member();
    test_brief_streak_not_remembered();
    test_forget_rejoin();
    setup(); now = UINT32_MAX - 6000U;
    run(24,1,false); assert(rc.auto_engaged); /* uptime wrap */
    puts("PASS auto normal/sustained/intermittent/outage/repair/recovery/hold/off/exclusion/roster-poor/relayed-outage/noisy-exit/pooled/new-weak/brief-streak/forget/leave/wrap");
}
