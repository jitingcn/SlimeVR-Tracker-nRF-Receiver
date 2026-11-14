#include <assert.h>
#include <stdio.h>
#include "grants_support.h"
static struct relay_coord rc;
int main(void)
{
	const struct relay_geometry *g = relay_geometry_current();
	uint16_t members = (uint16_t)((1U << g->direct) - 1U);
	relay_coord_init(&rc);
	rc.v3_requested = true;
	for (unsigned i = 0; i < g->direct; i++) {
		relay_coord_note_caps(&rc, i, RELAY_INFO_CAPS);
	}
	assert(relay_coord_v3_ready(&rc, members));
	struct relay_control c
		= {.profile = 1, .members = members, .expected = members, .repair_count = g->repairs, .frame = 10};
	for (unsigned i = 0; i < RELAY_MAX_TRACKERS; i++) {
		rc.maintenance_ms[i] = rc.report_ms[i] = 5000;
	}
	rc.maintenance_ms[0] = 0;
	struct rc_grant_plan plan;
	relay_coord_plan_grants(&rc, &c, &plan, 5000);
	relay_coord_finish_grants(&rc, &c, &plan);
	assert(!!(c.flags & RELAY_CONTROL_MAINTENANCE) == (g->repairs >= 2U));
	if (g->repairs >= 2U) {
		assert(c.grants[0].owner == RELAY_OWNER_MAINTENANCE);
	}
	if (g->repairs > 1) {
		assert(c.grants[1].owner == RELAY_OWNER_MAINTENANCE);
	}
	for (unsigned i = g->repairs; i < RELAY_V3_REPAIR_MAX; i++) {
		assert(c.grants[i].owner == RELAY_OWNER_IDLE);
	}
	uint8_t wire[72];
	struct relay_control parsed;
	relay_control_encode(wire, &c);
	assert(relay_control_decode(wire, sizeof(wire), &parsed));
	for (unsigned frame = 0; frame < 10000; frame++) {
		assert(relay_frame_start(0, frame) == (uint32_t)((uint64_t)frame * g->numerator / g->denominator));
	}
	puts("Configured repair count, direct capacity, slot width, TPS and maintenance passed");
}
