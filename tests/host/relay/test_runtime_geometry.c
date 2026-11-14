#include <assert.h>
#include <stdio.h>
#include "relay_coord.h"

int main(void)
{
	struct relay_runtime_settings s = {16, 1, 16, 100, 120, 0, 0};
	assert(relay_runtime_validate(&s, false));
	s.max_repairs = 5;
	assert(relay_runtime_validate(&s, false) && s.max_repairs == 5);
	s.max_trackers = 11;
	s.max_repairs = 5;
	assert(relay_runtime_validate(&s, false) && s.max_repairs == 5);
	s.slot_ticks = 15;
	assert(!relay_runtime_validate(&s, false));
	assert(relay_runtime_validate(&s, true));
	s.slot_ticks = 14;
	assert(!relay_runtime_validate(&s, true));
	s = (struct relay_runtime_settings){16, 1, 16, 120, 120, 0, 0};
	assert(relay_runtime_validate(&s, false));
	for (unsigned direct = 1; direct <= 16; direct++) {
		for (unsigned r = 0; r <= 5; r++) {
			uint32_t total = 0;
			for (unsigned f = 0; f < 120; f++) {
				unsigned period = relay_runtime_period(f, direct, r, 16, 120);
				assert(period >= (direct + 1U + r) * 16U);
				total += period;
			}
			if ((direct + 1U + r) * 16U <= 273U) {
				assert(total == 32768U);
			}
		}
	}
	for (unsigned current = 0; current <= 5; current++) {
		for (unsigned next = 0; next <= 5; next++) {
			for (unsigned next2 = 0; next2 <= 5; next2++) {
				struct relay_runtime_anchor a = {0};
				uint32_t start;
				assert(!relay_runtime_start(&a, 0, &start));
				relay_runtime_anchor_set(&a, 119, 0xfffffff0U, 16, current, next, next2, 16, 120);
				assert(relay_runtime_start(&a, 120, &start));
				assert(start == a.start + relay_runtime_period(119, 16, current, 16, 120));
				assert(relay_runtime_start(&a, 121, &start));
				assert(start == a.next_start + relay_runtime_period(120, 16, next, 16, 120));
				assert(relay_runtime_start(&a, 122, &start));
				assert(start == a.following_start + relay_runtime_period(121, 16, next2, 16, 120));
				assert(!relay_runtime_start(&a, 123, &start));
				uint32_t cutoff = a.third_start + 16U * 16U;
				assert(relay_runtime_direct_granted(&a, cutoff - 1U));
				assert(!relay_runtime_direct_granted(&a, cutoff));
				assert(!relay_runtime_direct_granted(&a, cutoff + 1000U));
			}
		}
	}
	for (unsigned repairs = 0; repairs <= 5; repairs++) {
		struct relay_control c = {.repair_count = repairs};
		struct rc_grant_plan p = {.urgent = true, .maintenance = 0, .reporter = -1};
		uint8_t wire[RELAY_CONTROL_LEN] = {0};
		memset(wire + RELAY_CONTROL_OFF_OWNERS, RELAY_OWNER_IDLE, RELAY_V3_REPAIR_MAX);
		relay_coord_finish_frozen(&c, &p, wire);
		for (unsigned slot = repairs; slot < RELAY_V3_REPAIR_MAX; slot++) {
			assert(wire[RELAY_CONTROL_OFF_OWNERS + slot] == RELAY_OWNER_IDLE);
		}
		assert(repairs || !(c.flags & RELAY_CONTROL_MAINTENANCE));
	}
	struct relay_control overlap = {.repair_count = 2, .missing = {0, 0, 4}};
	struct rc_grant_plan maintenance
		= {.urgent = true,
		   .maintenance = 0,
		   .reporter = -1,
		   .emergency_child = 4,
		   .emergency_helper = 1,
		   .emergency_coverage = 4};
	uint8_t wire[RELAY_CONTROL_LEN] = {0};
	memset(wire + RELAY_CONTROL_OFF_OWNERS, RELAY_OWNER_IDLE, RELAY_V3_REPAIR_MAX);
	relay_coord_finish_frozen(&overlap, &maintenance, wire);
	assert(
		wire[RELAY_CONTROL_OFF_OWNERS] == RELAY_OWNER_MAINTENANCE
		&& wire[RELAY_CONTROL_OFF_OWNER_SECOND] == RELAY_OWNER_MAINTENANCE
	);
	assert(
		relay_get_be16(wire + RELAY_CONTROL_OFF_CHILDREN) == 0
		&& relay_get_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2) == 0
	);
	puts("Runtime validation, rational cap, variable R, missed B and tick rollover passed");
}
