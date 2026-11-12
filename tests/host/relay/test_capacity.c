/* Wire IDs remain 0..15 even when local coordinator storage ends at ID 9. */
#include "relay_coord.h"
#include <assert.h>
#include <string.h>

int main(void)
{
    _Static_assert(RC_MAX == 10, "exercise reduced capacity");
    _Static_assert(RELAY_MAX_TRACKERS == 16, "wire capacity unchanged");
    struct relay_coord rc, before;
    relay_coord_init(&rc);
    rc.v3_requested = true;
    for (unsigned id = 0; id < RC_MAX; id++)
        relay_coord_note_caps(&rc, id, (RELAY_V3_VERSION << RELAY_CAP_VERSION_SHIFT) | RELAY_CAP_HELPER);
    assert(relay_coord_v3_ready(&rc, 0x3ff));
    for (unsigned id = 10; id < 16; id++) {
        assert(!relay_coord_v3_ready(&rc, (1U << id) | 0x3ff));
        before = rc;
        relay_coord_note_caps(&rc, id, 0xff);
        relay_coord_note_frame(&rc, id, RC_DIRECT, RC_SEQ_NORMAL, 0, 60, 100);
        relay_coord_note_bundle(&rc, id, true);
        relay_coord_note_trailer_sent(&rc, id, 0xffff, 100);
        relay_coord_forget(&rc, id, 100);
        relay_coord_pin(&rc, id, true);
        relay_coord_receive(&rc, id, 100);
        assert(!relay_coord_publish(&rc, id, 100));
        assert(!relay_coord_was_published(&rc, id, 100));
        assert(!relay_coord_skipped(&rc, id, 100));
        relay_coord_capture(&rc, id, 0, 60, 100);
        relay_coord_capture(&rc, 0, id, 60, 100);
        relay_coord_expire_peer(&rc, id, 0, 100);
        relay_coord_expire_peer(&rc, 0, id, 100);
        assert(!relay_coord_is_capable(&rc, id));
        assert(!relay_coord_is_relayed(&rc, id));
        assert(!relay_coord_direct_ready(&rc, id, 100));
        assert(relay_coord_forward_slot(&rc, id, 0xffff) == RELAY_NO_SLOT);
        struct relay_pong_trailer trailer;
        relay_coord_trailer(&rc, id, 1, 0xffff, &trailer);
        assert(!trailer.children_mask && trailer.forward_slot == RELAY_NO_SLOT);
        struct relay_control c = {.members = 0xffff, .repair_count = 1};
        c.grants[0].owner = id;
        struct relay_peer_report r = {.helper = id, .count = 1};
        r.entries[0].child = id;
        assert(!relay_coord_report(&rc, &c, 0, &r, 100));
        c.grants[0].owner = r.helper = 0;
        assert(relay_coord_report(&rc, &c, 0, &r, 100));
        assert(memcmp(&rc, &before, sizeof(rc)) == 0);
    }
    /* Malformed saved masks/helper nibbles cannot index shortened arrays. */
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
    /* ID 9's backup occupies wire bit 25, even with ten local tracker cells.
     * Cover both static pools and the five-age demand allocator. */
    for (unsigned mode = 0; mode < 3; mode++) {
        relay_coord_init(&rc);
        rc.cache_frames = mode == 2 ? 5 : 3;
        c = (struct relay_control){.members = 0x3ff, .expected = 1U << 9,
            .frame = 20, .repair_count = mode == 0 ? 3 : 5, .on_demand = mode == 2};
        for (unsigned h = 0; h < RC_MAX; h++) rc.t[h].direct_permille = 1000;
        for (unsigned age = 0; age < rc.cache_frames; age++) {
            struct rc_watch *w = &rc.watch[(20 - age) % rc.cache_frames];
            w->frame = 20 - age;
            w->members = 0x3ff;
            w->primary_valid = w->backup_valid = 1U << 9;
            w->helpers[9] = 0x10;
        }
        relay_coord_plan_watch(&rc, &c);
        relay_coord_plan_grants(&rc, &c, &plan, 0);
        uint8_t wire[72];
        for (unsigned age = 0; age < rc.cache_frames; age++)
            relay_control_missing(&c, age) = 1U << 9;
        relay_control_encode(wire, &c);
        relay_coord_finish_frozen(&c, &plan, wire);
        relay_coord_read_grants(&c, wire);
        assert(c.grants[0].children == (1U << 9));
        assert(c.grants[0].owner < RC_MAX);
        relay_coord_commit_grants(&rc, &c, &plan);
    }
    return 0;
}
