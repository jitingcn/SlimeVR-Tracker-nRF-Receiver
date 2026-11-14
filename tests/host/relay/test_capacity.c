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
    rc.v3_requested = true;
    for (unsigned id = 0; id < RC_MAX; id++)
        relay_coord_note_caps(&rc, id, (RELAY_V3_VERSION << RELAY_CAP_VERSION_SHIFT) | RELAY_CAP_HELPER);
    assert(relay_coord_v3_ready(&rc, 0xffff));
    assert(relay_coord_v3_ready(&rc, 0xffff));
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
        rc.watch[i].frame = 10 - i;
        rc.watch[i].members = 0xffff;
        rc.watch[i].primary_valid = rc.watch[i].backup_valid = 0xffff;
        memset(rc.watch[i].helpers, 0xff, sizeof(rc.watch[i].helpers));
    }
    struct rc_grant_plan plan;
    relay_coord_plan_watch(&rc, &c);
    relay_coord_plan_grants(&rc, &c, &plan, 5000);
    plan.maintenance = plan.reporter = 15;
    c.flags |= RELAY_CONTROL_MAINTENANCE;
    c.grants[0].owner = 0;
    c.grants[0].children = 0xffff;
    relay_coord_commit_grants(&rc, &c, &plan);
    /* ID 15 exercises the top backup lane in both static pools and demand. */
    for (unsigned mode = 0; mode < 3; mode++) {
        relay_coord_init(&rc);
        rc.cache_frames = mode == 2 ? 5 : 3;
        c = (struct relay_control){.members = 0xffff, .expected = 1U << 15,
            .frame = 20, .repair_count = mode == 0 ? 3 : 5, .on_demand = mode == 2};
        for (unsigned h = 0; h < RC_MAX; h++) rc.t[h].direct_permille = 1000;
        for (unsigned age = 0; age < rc.cache_frames; age++) {
            struct rc_watch *w = &rc.watch[(20 - age) % rc.cache_frames];
            w->frame = 20 - age;
            w->members = 0xffff;
            w->primary_valid = w->backup_valid = 1U << 15;
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
        assert(c.grants[0].children == (1U << 15));
        assert(c.grants[0].owner < RC_MAX);
        relay_coord_commit_grants(&rc, &c, &plan);
    }
    return 0;
}
