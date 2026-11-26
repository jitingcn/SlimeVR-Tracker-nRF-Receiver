/* Every chip admits high IDs and plans all wire lanes. */
#include "relay_coord.h"
#include <assert.h>
#include <string.h>

int main(void)
{
    _Static_assert(RC_MAX == 16, "all chips cover every tracker ID");
    _Static_assert(RELAY_MAX_TRACKERS == 16, "wire capacity unchanged");
    struct relay_coord rc;
    relay_coord_init(&rc);
    rc.enabled = true;
    for (unsigned id = 0; id < RC_MAX; id++)
        relay_coord_note_caps(&rc, id, (RELAY_PROTOCOL_VERSION << RELAY_CAP_VERSION_SHIFT) | RELAY_CAP_HELPER);
    assert(relay_coord_ready(&rc, 0xffff));
    assert(relay_coord_requested(&rc, RELAY_MODE_FORCE, 0xffff));
    uint8_t caps = (RELAY_PROTOCOL_VERSION << RELAY_CAP_VERSION_SHIFT) | RELAY_CAP_HELPER;
    relay_coord_note_caps(&rc, 15, caps | RELAY_CAP_NATIVE_TEST);
    assert(relay_coord_capable(&rc, 0xffff));
    assert(!relay_coord_requested(&rc, RELAY_MODE_FORCE, 0xffff));
    assert(relay_coord_requested(&rc, RELAY_MODE_FORCE, 0x7fff));
    relay_coord_note_caps(&rc, 15, 0);
    assert(!relay_coord_is_capable(&rc, 15));
    assert(!(rc.native_test_members & (1U << 15)));
    relay_coord_note_caps(&rc, 15, caps | RELAY_CAP_NATIVE_TEST);
    relay_coord_forget(&rc, 15, 0);
    assert(!relay_coord_is_capable(&rc, 15));
    assert(!(rc.native_test_members & (1U << 15)));
    relay_coord_note_caps(&rc, 15, caps);
    assert(relay_coord_requested(&rc, RELAY_MODE_FORCE, 0xffff));
    for (unsigned id = 10; id < 16; id++) {
        assert(relay_coord_is_capable(&rc, id));
        relay_coord_note_frame(&rc, id, RC_DIRECT, RC_SEQ_NORMAL, 0, 60, 100);
        relay_coord_receive(&rc, id, 100);
        assert(relay_coord_publish(&rc, id, 100));
        assert(relay_coord_was_published(&rc, id, 100));
        relay_coord_capture(&rc, id, 0, 60, 100);
        relay_coord_capture(&rc, 0, id, 60, 100);
        assert(rc.peer[id][0].rssi_x8 && rc.peer[0][id].rssi_x8);
    }
    /* Full masks and the highest helper nibble stay in bounds. */
    struct relay_control c = {.members = 0xffff, .expected = 0xffff, .frame = 10, .repair_count = 3};
    for (unsigned i = 0; i < 3; i++) {
        struct rc_watch *w = &rc.watch[(10 - i) % 3];
        w->frame = 10 - i;
        w->members = 0xffff;
        w->primary_valid = w->backup_valid = 0xffff;
        memset(w->helpers, 0xff, sizeof(w->helpers));
    }
    struct rc_grant_plan plan;
    relay_coord_plan_watch(&rc, &c);
    relay_coord_plan_grants(&rc, &c, &plan, 5000);
    assert(plan.valid == 0x7);
    plan.maintenance = plan.reporter = 15;
    c.flags |= RELAY_CONTROL_MAINTENANCE;
    c.grants[0].owner = 0;
    c.grants[0].children = 0xffff;
    relay_coord_commit_grants(&rc, &c, &plan);
    /* ID 15 exercises the top backup lane in both static pools and demand. */
    for (unsigned mode = 0; mode < 3; mode++) {
        for (unsigned backup_only = 0; backup_only < 2; backup_only++) {
            relay_coord_init(&rc);
            rc.cache_frames = mode == 2 ? 5 : 3;
            c = (struct relay_control){.members = 0xffff, .expected = 1U << 15,
                .frame = 20, .repair_count = mode == 0 ? 3 : 5, .on_demand = mode == 2};
            for (unsigned h = 0; h < RC_MAX; h++) rc.t[h].direct_permille = 1000;
            for (unsigned age = 0; age < rc.cache_frames; age++) {
                struct rc_watch *w = &rc.watch[(20 - age) % rc.cache_frames];
                w->frame = 20 - age;
                w->members = 0xffff;
                w->primary_valid = backup_only ? 0 : (1U << 15);
                w->backup_valid = 1U << 15;
                w->helpers[15] = 0x10;
            }
            relay_coord_plan_watch(&rc, &c);
            relay_coord_plan_grants(&rc, &c, &plan, 0);
            uint8_t wire[72];
            for (unsigned age = 0; age < rc.cache_frames; age++)
                relay_control_missing(&c, age) = 1U << 15;
            relay_control_encode(wire, &c);
            relay_coord_finish_frozen(&c, &plan, wire);
            relay_coord_read_grants(&c, wire);
            if (backup_only) {
                bool saw_backup = false;
                for (unsigned s = 0; s < c.repair_count; s++) {
                    if (c.grants[s].children) {
                        assert(c.grants[s].owner == 1);
                        assert(c.grants[s].children == (1U << 15));
                        saw_backup = true;
                    }
                }
                assert(saw_backup);
            } else {
                assert(c.grants[0].children == (1U << 15));
                assert(c.grants[0].owner < RC_MAX);
            }
            relay_coord_commit_grants(&rc, &c, &plan);
        }
    }
    return 0;
}
