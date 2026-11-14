#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "relay_coord.h"
#include "relay_demand.h"

static void policy(void)
{
	struct relay_runtime_settings s = {11, 5, 16, 100, 120, 0, 0};
	struct relay_demand d;
	relay_demand_reset(&d, 100, 0);
	uint8_t current = d.queued[0], next = d.queued[1];
	relay_demand_update(&d, 100, 3, &s);
	assert(d.queued[0] == current && d.queued[1] == next && d.queued[2] == 3);
	for (unsigned f = 101; f < 130; f++) {
		current = d.queued[1];
		next = d.queued[2];
		relay_demand_update(&d, f, 0, &s);
		assert(d.queued[0] == current && d.queued[1] == next && d.target == 3);
	}
	relay_demand_update(&d, 130, 0, &s);
	assert(d.target == 2);
	relay_demand_update(&d, 131, 1, &s);
	assert(d.target == 2 && d.quiet == 0);
	for (unsigned f = 132; f <= 221; f++) {
		relay_demand_update(&d, f, 0, &s);
	}
	assert(d.target == 0);
	for (unsigned f = 222; f < 320; f++) {
		relay_demand_update(&d, f, 0, &s);
		assert(d.queued[2] == ((f + 2) % 8 == 0 ? 2 : 0));
	}
	s.max_trackers = 16;
	assert(relay_runtime_validate(&s, false) && s.max_repairs == 5);
	s = relay_runtime_for_members(s, 16);
	relay_demand_update(&d, 320, 16, &s);
	assert(d.target == 3);
	s.repair_floor = 6;
	assert(!relay_runtime_validate(&s, false));
	for (unsigned f = 321; f < 500; f++) {
		relay_demand_update(&d, f, 16, &(struct relay_runtime_settings){16, 3, 16, 100, 120, 1, 0});
		for (unsigned k = 0; k < 3; k++) {
			assert(relay_runtime_period(f + k, 16, d.queued[k], 16, 120) <= 32768 / 100);
		}
	}
	s.repair_floor = 0;
	relay_demand_reset(&d, UINT32_MAX - 1U, 0);
	relay_demand_update(&d, UINT32_MAX - 1U, 2, &s);
	current = d.queued[1];
	next = d.queued[2];
	relay_demand_advance(&d, UINT32_MAX, &s);
	assert(d.queued[0] == current && d.queued[1] == next);
	current = d.queued[1];
	next = d.queued[2];
	relay_demand_advance(&d, 0, &s);
	assert(d.queued[0] == current && d.queued[1] == next);
	puts("PASS demand: announced queue, rollover, hysteresis, step-down, maintenance and budget clamp");
}

static void wire(void)
{
	for (unsigned r = 0; r <= 5; r++) {
		for (unsigned owner = 0; owner <= 16; owner++) {
			struct relay_control c
				= {.direct_count = 11,
				   .slot_ticks = 16,
				   .tps_max = 120,
				   .members = 0x7ff,
				   .expected = 0x7ff,
				   .profile = 1,
				   .repair_count = r,
				   .on_demand = true,
				   .maintenance_owner = RELAY_OWNER_IDLE};
			for (unsigned i = 0; i < 5; i++) {
				c.grants[i].owner = RELAY_OWNER_IDLE;
			}
			c.maintenance_owner = owner ? owner - 1 : RELAY_OWNER_IDLE;
			uint8_t buf[72], expected[72];
			assert(relay_control_encode(buf, &c) == 72);
			struct relay_control_crc_plan crc = relay_control_plan_crc(buf);
			c.missing[0] = 3;
			c.missing[1] = 7;
			c.missing[2] = 11;
			relay_control_missing(&c, 3) = 13;
			relay_control_missing(&c, 4) = 17;
			c.maintenance_owner = owner == 16 ? RELAY_OWNER_IDLE : owner;
			relay_control_set_next2_repairs(&c, (r + 1) % 6);
			relay_control_finish_encode(buf, &c, crc, true);
			assert(relay_control_encode(expected, &c) == 72);
			assert(!memcmp(buf, expected, 72));
			struct relay_control decoded;
			assert(relay_control_decode(buf, 72, &decoded));
			assert(
				decoded.missing_extra[0] == c.missing_extra[0] && decoded.missing_extra[1] == c.missing_extra[1]
				&& decoded.on_demand && decoded.direct_count == 11 && decoded.slot_ticks == 16
				&& decoded.maintenance_owner == c.maintenance_owner
				&& !memcmp(decoded.missing, c.missing, sizeof(c.missing))
			);
			buf[1] = RELAY_V3_VERSION + 1U;
			buf[71] = relay_crc8(7, buf, 71);
			assert(!relay_control_decode(buf, 72, &decoded));
		}
	}
	puts("PASS demand: v7 five-mask roundtrip and frozen CRC, all R and maintenance owners");
}

static void late_direct(void)
{
	for (unsigned repairs = 1; repairs <= 5; repairs++) {
		struct relay_coord rc = {.cache_frames = 5, .learn_members = 0x7ff};
		struct relay_control c
			= {.direct_count = 11,
			   .slot_ticks = 16,
			   .tps_max = 120,
			   .members = 0x7ff,
			   .expected = 0x7ff,
			   .profile = 1,
			   .repair_count = repairs,
			   .on_demand = true,
			   .frame = 100};
		for (unsigned ch = 0; ch < 11; ch++) {
			rc.t[ch].direct_permille = 1000;
			rc.watch[0].helpers[ch] = (ch + 3) % 11;
		}
		rc.watch[0].frame = 100;
		rc.watch[0].members = 0x7ff;
		rc.watch[0].primary_valid = 0x7ff;
		struct rc_grant_plan plan;
		relay_coord_plan_grants(&rc, &c, &plan, 0);
		uint8_t wire[72];
		relay_control_encode(wire, &c);
		for (unsigned age = 0; age < 5; age++) {
			relay_control_missing(&c, age) = 0;
		}
		c.missing[0] = 1U << 9;
		relay_coord_finish_frozen(&c, &plan, wire);
		relay_coord_read_grants(&c, wire);
		bool found = false;
		for (unsigned i = 0; i < repairs; i++) {
			if (c.grants[i].children & (1U << 9)) {
				found = true;
			}
		}
		assert(found);
		rc.t[1].direct_permille = 949;
		relay_coord_plan_grants(&rc, &c, &plan, 0);
		assert(!(plan.demand_eligible[0] & (1U << 9)));
		rc.t[1].direct_permille = 950;
		relay_coord_plan_grants(&rc, &c, &plan, 0);
		assert(plan.demand_eligible[0] & (1U << 9));
	}
	puts("PASS demand: late direct miss refills a vacated planned grant at age zero for every open R");
}

static void maintenance_debt(void)
{
	struct relay_coord rc = {.cache_frames = 5, .learn_members = 0x7ff};
	struct relay_control c
		= {.direct_count = 11,
		   .slot_ticks = 16,
		   .tps_max = 120,
		   .members = 0x7ff,
		   .expected = 0x7ff,
		   .profile = 1,
		   .repair_count = 2,
		   .on_demand = true,
		   .frame = 56};
	for (unsigned h = 0; h < 11; h++) {
		rc.t[h].direct_permille = 1000;
	}
	for (unsigned age = 0; age < 5; age++) {
		unsigned i = (56 - age) % 5;
		rc.watch[i] = (struct rc_watch){.frame = 56 - age, .members = 0x7ff, .primary_valid = 1U << 9};
		rc.watch[i].helpers[9] = 1;
		rc.receipt[i] = (struct rc_receipt){.frame = 56 - age, .received = 0x7ff};
	}
	rc.receipt[52 % 5].received &= ~(1U << 9);
	struct rc_grant_plan plan;
	relay_coord_plan_grants(&rc, &c, &plan, 5000);
	assert(!(c.flags & RELAY_CONTROL_MAINTENANCE) && plan.maintenance < 0);
	bool found = false;
	for (unsigned i = 0; i < 2; i++) {
		found |= !!(c.grants[i].children & (1U << 9));
	}
	assert(found);
	rc.receipt[52 % 5].received = 0x7ff;
	relay_coord_plan_grants(&rc, &c, &plan, 5000);
	assert(
		(c.flags & RELAY_CONTROL_MAINTENANCE) && c.grants[0].owner == RELAY_OWNER_MAINTENANCE
		&& c.grants[1].owner == RELAY_OWNER_MAINTENANCE
	);
	puts("PASS demand: maintenance yields to aged debt when both cannot fit, then resumes with two slots");
}

/* Deterministic capacity simulation using the production planner, frozen
 * allocator and immutable queue. Helpers capture every assigned pose; repair
 * packet delivery is perfect. Results describe capacity under those assumptions. */
static unsigned state = 42;
static unsigned random32(void)
{
	state ^= state << 13;
	state ^= state >> 17;
	state ^= state << 5;
	return state;
}
static void simulate(unsigned members, unsigned mode, unsigned trace, unsigned frames)
{
	state = 42;
	struct relay_coord rc = {0};
	struct relay_demand d;
	struct relay_runtime_settings settings
		= {members,
		   5,
		   16,
		   100,
		   120,
		   mode < 2    ? mode
		   : mode == 2 ? 1
		   : mode == 3 ? 3
					   : 5,
		   0};
	assert(relay_runtime_validate(&settings, false));
	if (mode >= 2) {
		settings.max_repairs = settings.repair_floor;
	}
	unsigned ages = mode < 2 ? 5 : 3;
	rc.cache_frames = ages;
	if (trace == 6) {
		for (unsigned h = 0; h < members; h++) {
			rc.maintenance_ms[h] = UINT32_MAX - 5000;
		}
	}
	relay_demand_reset(&d, 0, settings.repair_floor);
	uint16_t received[5] = {0}, lost[5] = {0};
	unsigned occupancy[6] = {0};
	unsigned long delivered = 0, total = 0, ticks = 0, onset = 0;
	uint16_t mask = (1U << members) - 1U;
	for (unsigned f = 0; f < frames + 5; f++) {
		relay_demand_advance(&d, f, &settings);
		struct relay_control c
			= {.direct_count = members,
			   .slot_ticks = 16,
			   .tps_max = 120,
			   .members = mask,
			   .expected = mask,
			   .profile = 1,
			   .frame = f,
			   .repair_count = mode < 2 ? d.queued[0] : relay_demand_slots(f, settings.max_repairs),
			   .on_demand = mode < 2};
		unsigned idx = f % 5;
		received[idx] = mask;
		lost[idx] = 0;
		for (unsigned ch = 0; ch < members; ch++) {
			bool drop = trace < 3  ? random32() % 10000 < (unsigned[]){50, 200, 500}[trace]
					  : trace == 3 ? (f % 200 >= 50 && f % 200 < 55 && (ch == 3 || ch == 9))
					  : trace == 4 ? ch == 3
								   : (f == (trace == 6 ? 52U : 50U) && (ch == 3 || ch == 9));
			if (drop) {
				received[idx] &= ~(1U << ch);
				lost[idx] |= 1U << ch;
			}
			rc.t[ch].direct_permille = trace == 4 && ch == 3 ? 0 : 1000;

			c.primary_valid |= 1U << ch;
			c.backup_valid |= 1U << ch;
			c.helpers[ch] = ((ch + 3) % members) | (((ch + 6) % members) << 4);
		}
		for (unsigned age = 0; age < ages; age++) {
			unsigned wi = (f - age) % ages;
			rc.watch[wi]
				= (struct rc_watch){.frame = f - age, .members = mask, .primary_valid = mask, .backup_valid = mask};
			memcpy(rc.watch[wi].helpers, c.helpers, sizeof(c.helpers));
			rc.receipt[wi]
				= (struct rc_receipt){.frame = f - age, .received = f >= age ? received[(f - age) % 5] : mask};
		}
		rc.receipt[f % ages].received &= (1U << (members - 8U)) - 1U;
		rc.learn_members = mask;
		struct rc_grant_plan plan;
		relay_coord_plan_grants(&rc, &c, &plan, f * 8);
		uint16_t debt = 0;
		for (unsigned age = 0; age < ages; age++) {
			relay_control_missing(&c, age) = mask & ~(f >= age ? received[(f - age) % 5] : mask);
			if (mode < 2) {
				debt |= relay_control_missing(&c, age) & plan.demand_eligible[age];
			}
		}
		if (mode < 2) {
			uint8_t a = d.queued[0], b = d.queued[1];
			relay_demand_update(&d, f, relay_mask_count(debt), &settings);
			assert(a == d.queued[0] && b == d.queued[1]);
		}

		relay_control_set_next_repairs(&c, mode < 2 ? d.queued[1] : relay_demand_slots(f + 1, settings.max_repairs));
		relay_control_set_next2_repairs(&c, mode < 2 ? d.queued[2] : relay_demand_slots(f + 2, settings.max_repairs));
		uint8_t buf[72];
		assert(relay_control_encode(buf, &c) == 72);
		struct relay_control_crc_plan crc = relay_control_plan_crc(buf);
		relay_coord_finish_frozen(&c, &plan, buf);
		relay_control_finish_encode(buf, &c, crc, false);
		relay_coord_read_grants(&c, buf);
		struct relay_control decoded;
		assert(relay_control_decode(buf, 72, &decoded));
		assert(decoded.repair_count == c.repair_count && decoded.on_demand == c.on_demand);
		relay_coord_commit_grants(&rc, &c, &plan);
		/* Helpers receive feedback only in the next B. A second holder spends
		 * capacity on a duplicate even when the receiver heard the first holder. */
		uint16_t selected[RC_MAX][5] = {{0}};
		for (unsigned slot = 0; slot < c.repair_count; slot++) {
			const struct relay_grant *g = &c.grants[slot];
			unsigned capacity = 3;
			uint16_t blocked = 0;
			if (g->owner >= members) {
				continue;
			}
			bool delivered_packet = !((trace == 5 && (f == 52 || f == 53)) || (trace == 6 && (f == 54 || f == 55)));
			for (int age = ages - 1; age >= 0 && capacity; age--) {
				uint16_t included = 0;
				if (f >= (unsigned)age) {
					uint16_t missing
						= relay_control_missing(&c, age) & g->children & ~selected[g->owner][age] & ~blocked;
					while (missing && capacity) {
						unsigned ch = __builtin_ctz(missing);
						missing &= missing - 1U;
						selected[g->owner][age] |= 1U << ch;
						included |= 1U << ch;
						capacity--;
						if (delivered_packet) {
							received[(f - age) % 5] |= 1U << ch;
						}
						if (delivered_packet && trace == 3 && f % 200 >= 52 && f % 200 <= 54 && mode < 2) {
							onset++;
						}
					}
				}
				if (!(age == (int)ages - 1 && c.repair_count <= 3 && slot + 1 == c.repair_count)) {
					blocked |= relay_control_missing(&c, age) & ~included;
				}
			}
		}

		if (f < frames) {
			occupancy[c.repair_count]++;
			ticks += relay_runtime_period(f, members, c.repair_count, 16, 120);
		}
		if (f >= 4 && f - 4 < frames) {
			unsigned ri = (f - 4) % 5;
			delivered += relay_mask_count(received[ri]);
			total += members;
		}
	}
	if (trace == 3 && mode < 2) {
		assert(onset > 0 && delivered == total);
	}
	if (trace == 5 || trace == 6) {
		assert(delivered == total);
	}
	printf("%u,%u,%u,%.6f,%.9f", members, mode, trace, (double)frames * 32768 / ticks, (double)delivered / total);
	for (unsigned i = 0; i < 6; i++) {
		printf(",%.6f", (double)occupancy[i] / frames);
	}
	puts("");
}
int main(int argc, char **argv)
{
	for (unsigned mask = 0; mask <= UINT16_MAX; mask++) {
		assert(relay_demand_count(mask) == relay_mask_count(mask));
	}
	policy();
	wire();
	late_direct();
	maintenance_debt();
	simulate(11, 0, 5, 200);
	simulate(11, 1, 5, 200);
	simulate(11, 0, 6, 200);
	simulate(11, 1, 6, 200);
	puts("PASS demand: simultaneous burst onset survives failed age-two and age-three attempts, delivered by age four");
	if (argc > 1) {
		unsigned frames = strtoul(argv[1], 0, 10);
		puts("members,mode,trace,tps,within5,R0,R1,R2,R3,R4,R5");
		for (unsigned n = 11; n <= 16; n += 5) {
			for (unsigned mode = 0; mode < 5; mode++) {
				for (unsigned trace = 0; trace < 5; trace++) {
					if (n != 16 || mode != 4) {
						simulate(n, mode, trace, frames);
					}
				}
			}
		}
	}
}
