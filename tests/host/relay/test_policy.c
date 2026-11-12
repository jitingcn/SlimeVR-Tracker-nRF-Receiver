#include <assert.h>
#include <stdio.h>
#include "relay_demand.h"
int main(void)
{
	for (unsigned target = 100; target <= 120; target += 20) {
		for (unsigned mode = 0; mode <= 2; mode++) {
			struct relay_runtime_settings settings = {16, 5, 16, target, 120, 5, mode};
			assert(relay_runtime_validate(&settings, false));
			uint32_t last = 0;
			for (unsigned t = 1; t <= 16; t++) {
				struct relay_runtime_settings p = relay_runtime_for_members(settings, t);
				unsigned nominal = (target == 120 ? 17 : 20) - t - 1;
				nominal = nominal < 1 ? 1 : nominal > 5 ? 5 : nominal;
				unsigned budget = (target == 120 ? 17 : 20) > t + 1 ? (target == 120 ? 17 : 20) - t - 1 : 1;
				unsigned m = nominal >= 3 && (mode != 1 || budget >= 4) ? mode : 0, r = nominal - (m != 0);
				if (m && r > budget - relay_second_b_slots(m)) {
					r = budget - relay_second_b_slots(m);
				}
				assert(p.max_repairs == r && p.repair_floor == r && p.second_b == m);
				struct relay_geometry g;
				assert(relay_geometry_make_b(&g, t, r, 16, 120, m));
				uint32_t sum = 0;
				for (unsigned f = 0; f < 120; f++) {
					unsigned actual = relay_demand_slots(f, r);
					unsigned period = relay_runtime_period(f, t, actual + relay_second_b_slots(m), 16, 120);
					assert(period >= (t + 1 + actual + relay_second_b_slots(m)) * 16);
					sum += period;
				}
				if (r >= 2) {
					assert(
						sum
						== (t + 1 + r + relay_second_b_slots(m) <= 17
								? 32768
								: 120 * (t + 1 + r + relay_second_b_slots(m)) * 16)
					);
				}
				if (t == 16 && target == 120) {
					assert(sum == 34800); /* 15 R2 maintenance frames, 105 R1 frames. */
				}
				printf(
					"target=%u T=%u mode=%u R=%u ticks/120=%u TPS=%.6f\n",
					target,
					t,
					m,
					r,
					sum,
					32768.0 * 120 / sum
				);
				/* A staged publication has value semantics: old geometry and queue remain immutable. */
				if (last) {
					struct relay_runtime_settings old = relay_runtime_for_members(settings, t - 1);
					assert(old.second_b != p.second_b || old.max_repairs >= p.max_repairs);
				}
				last = r;
			}
		}
	}
	for (unsigned demand = 0; demand <= 1; demand++) {
		for (unsigned mode = 1; mode <= 2; mode++) {
			for (unsigned width = 15; width <= 16; width++) {
				struct relay_control c
					= {.direct_count = 11,
					   .slot_ticks = width,
					   .tps_max = 120,
					   .repair_count = 4,
					   .members = 0x7ff,
					   .expected = 0x7ff,
					   .profile = 1,
					   .session = 123,
					   .epoch = 2,
					   .frame = 120,
					   .preamble_ticks = 0xfffffff0U,
					   .maintenance_owner = RELAY_OWNER_IDLE,
					   .on_demand = demand,
					   .second_b = mode};
				relay_control_set_next_repairs(&c, 3);
				relay_control_set_next2_repairs(&c, 2);
				for (unsigned i = 0; i < 5; i++) {
					c.grants[i].owner = RELAY_OWNER_IDLE;
				}
				uint8_t wire[72], repeat[72];
				assert(relay_control_encode(wire, &c) == 72);
				struct relay_control_crc_plan crc = relay_control_plan_crc(wire);
				relay_control_finish_encode(wire, &c, crc, false);
				assert(wire[71] == relay_crc8(7, wire, 71));
				struct relay_control a = {0}, b = {0};
				assert(relay_control_decode(wire, 72, &a));
				memcpy(repeat, wire, 72);
				repeat[0] = 0xec;
				relay_put_be32(repeat + 12, c.preamble_ticks + relay_second_b_offset(11, 4, width, mode));
				repeat[71] = relay_crc8(7, repeat, 71);
				assert(relay_control_decode(repeat, 72, &b));
				assert(!memcmp(&a, &b, sizeof(a))); /* padding zeroed below by compiler? */
				assert(a.second_b == mode && a.repair_count == 4 && a.preamble_ticks == c.preamble_ticks);
				struct relay_runtime_anchor anchor;
				relay_runtime_anchor_set(&anchor, c.frame, c.preamble_ticks, 11, 4, 3, 2, width, 120);
				relay_runtime_anchor_second_b(&anchor, 4, 3, 2, 120, mode);
				assert(
					anchor.next_start
					== anchor.start + relay_runtime_period(c.frame, 11, 4 + relay_second_b_slots(mode), width, 120)
				);
				assert(
					anchor.following_start
					== anchor.next_start
						   + relay_runtime_period(c.frame + 1, 11, 3 + relay_second_b_slots(mode), width, 120)
				);
				assert(
					anchor.third_start
					== anchor.following_start
						   + relay_runtime_period(c.frame + 2, 11, 2 + relay_second_b_slots(mode), width, 120)
				);
			}
		}
	}
	puts("Membership policy, maintenance, staged value snapshots and both immutable B copies pass");
}
