#include <assert.h>
#include <stdio.h>
#include "relay_coord.h"
int main(void)
{
	const struct relay_geometry *g = relay_geometry_current();
	assert(relay_plan_lead(0x7ff) == (g->direct > 11U ? 82U : 120U));
	assert(relay_plan_lead(0xffff) == 120U);
	assert(relay_geometry_valid(1, 0x7ff, 0x7ff, g->slot_ticks, g->repairs, g));
	assert(!relay_geometry_valid(1, 1, 2, g->slot_ticks, g->repairs, g));
	assert(relay_geometry_valid(1, 1, 1, g->slot_ticks, 0, g));
	struct relay_geometry decoded_geometry;
	assert(!relay_geometry_from_bootstrap(relay_geometry_code(11, 6), 16, &decoded_geometry));
	const struct relay_geometry cases[] = {{11, 5, 16, 120, 4096, 15, 0}, {11, 3, 15, 120, 4096, 15, 0}};
	for (unsigned row = 0; row < sizeof(cases) / sizeof(cases[0]); row++) {
		const struct relay_geometry *other = &cases[row];
		struct relay_layout_descriptor source
			= {.direct_count = 11,
			   .tps_max = other->tps,
			   .profile = 1,
			   .members = 0x7ff,
			   .expected = 0x7ff,
			   .slot_ticks = other->slot_ticks,
			   .repair_count = other->repairs},
			parsed;
		uint8_t wire[RELAY_LAYOUT_LEN], again[RELAY_LAYOUT_LEN];
		assert(relay_layout_encode(wire, &source) == sizeof(wire));
		assert(relay_layout_decode(wire, sizeof(wire), &parsed));
		assert(parsed.direct_count == 11 && parsed.tps_max == other->tps);
		assert(relay_layout_encode(again, &parsed) == sizeof(again) && !memcmp(wire, again, sizeof(wire)));
		parsed.direct_count++;
		assert(!relay_layout_encode(again, &parsed));
	}
	struct relay_layout_descriptor d
		= {.profile = 1, .members = 0x7ff, .expected = 0x7ff, .slot_ticks = g->slot_ticks, .repair_count = g->repairs},
		parsed;
	uint8_t wire[RELAY_LAYOUT_LEN];
	assert(relay_layout_encode(wire, &d) == sizeof(wire));
	assert(relay_layout_decode(wire, sizeof(wire), &parsed));
	wire[25]--;
	wire[29] = relay_crc8(7, wire, 29);
	assert(!relay_layout_decode(wire, sizeof(wire), &parsed));
	struct relay_control c
		= {.profile = 1,
		   .members = 0x7ff,
		   .expected = 0x7ff,
		   .repair_count = g->repairs,
		   .maintenance_owner = RELAY_OWNER_IDLE},
		decoded;
	for (unsigned i = 0; i < RELAY_V3_REPAIR_MAX; i++) {
		c.grants[i].owner = RELAY_OWNER_IDLE;
	}
	uint8_t control[RELAY_CONTROL_LEN];
	relay_control_encode(control, &c);
	assert(relay_control_decode(control, sizeof(control), &decoded));
	struct relay_coord rc;
	struct rc_grant_plan plan;
	relay_coord_init(&rc);
	c.members = c.expected = (uint16_t)((1U << g->direct) - 1U);
	relay_coord_plan_grants(&rc, &c, &plan, 5000);
	for (unsigned i = g->repairs; i < RELAY_V3_REPAIR_MAX; i++) {
		assert(c.grants[i].owner == RELAY_OWNER_IDLE);
	}
	relay_control_encode(control, &c);
	assert(relay_control_decode(control, sizeof(control), &decoded));
	assert(relay_member_slot(c.members, g->direct - 1U) == g->direct - 1U);

	assert((control[3] >> 3) + 1 == g->direct && control[68] == g->slot_ticks);
	assert(relay_get_be16(control + 69) == g->tps);
	control[69] = control[70] = 0;
	control[71] = relay_crc8(7, control, 71);
	assert(!relay_control_decode(control, sizeof(control), &decoded));
	for (unsigned f = 0; f < 100000; f++) {
		uint32_t start = relay_frame_start(0xffff0000U, f), end = relay_frame_start(0xffff0000U, f + 1);
		assert(relay_frame_at_tick(0xffff0000U, start) == f);
		assert(relay_frame_at_tick(0xffff0000U, end - 1) == f);
		assert(relay_frame_near(f, start, end) == f + 1);
		assert(
			end - start == g->numerator / g->denominator
			|| end - start == (g->numerator + g->denominator - 1U) / g->denominator
		);
	}
	assert(relay_frame_start(0, g->denominator) == g->numerator);
	puts("Layout wire, invalid geometry and exact frame math passed");
}
