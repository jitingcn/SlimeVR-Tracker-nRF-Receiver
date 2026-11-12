#include "relay_coord.h"
#include "relay_demand.h"
#ifndef RELAY_BENCH_SECOND_B
#define RELAY_BENCH_SECOND_B 0
#endif
#ifndef RELAY_BENCH_DEMAND
#define RELAY_BENCH_DEMAND 0
#endif
struct relay_demand demand_state;
struct relay_runtime_settings demand_settings;
#include <string.h>

struct relay_coord rc;
struct relay_control c;
struct rc_grant_plan plan;
uint8_t payload[RELAY_CONTROL_LEN];
struct relay_control_crc_plan crc;
static uint32_t state;

static uint32_t rnd(void)
{
	state ^= state << 13;
	state ^= state >> 17;
	state ^= state << 5;
	return state;
}

#ifndef RELAY_BENCH_MAX_MEMBERS
#define RELAY_BENCH_MAX_MEMBERS 16U
#endif
#define RELAY_BENCH_MIN_MEMBERS (RELAY_BENCH_MAX_MEMBERS < 11U ? RELAY_BENCH_MAX_MEMBERS : 11U)
void setup(unsigned seed)
{
	state = seed + 1;
	memset(&rc, 0, sizeof(rc));
	memset(&c, 0, sizeof(c));
	c.members
		= (1U << (RELAY_BENCH_MIN_MEMBERS + seed % (RELAY_BENCH_MAX_MEMBERS - RELAY_BENCH_MIN_MEMBERS + 1U))) - 1U;
	c.expected = c.members;
	c.on_demand = RELAY_BENCH_DEMAND;
	c.second_b = RELAY_BENCH_SECOND_B;
	rc.cache_frames = c.on_demand ? 5 : 3;
	demand_settings = (struct relay_runtime_settings){
		RELAY_BENCH_MAX_MEMBERS,
		5,
		RELAY_V3_SLOT_TICKS,
		100,
		RELAY_HOST_TPS_MAX,
		0,
		0
	};

	rc.learn_members = c.members;
	c.frame = 30 + seed / 6 % 10;
	relay_demand_reset(&demand_state, c.frame, c.second_b ? 2 : 0);
	demand_settings.repair_floor = c.second_b ? 2 : seed % 2U;
	demand_settings.max_repairs = c.second_b ? RELAY_V3_REPAIR_SLOTS : 5;
	demand_state.target = c.second_b ? 2 + seed % 3U : seed % 6U;
	demand_state.quiet = seed % 30U;
	c.repair_count = RELAY_V3_REPAIR_SLOTS;
	c.profile = RELAY_PROFILE_120HZ;
	c.direct_count = relay_mask_count(c.members);
	c.slot_ticks = RELAY_V3_SLOT_TICKS;
	c.tps_max = RELAY_HOST_TPS_MAX;
	relay_control_set_next_repairs(&c, c.second_b ? 2 + seed % 3U : seed % 6U);
	relay_control_set_next2_repairs(&c, c.second_b ? 2 + (seed / 6U) % 3U : (seed / 6U) % 6U);
	c.primary_valid = c.backup_valid = c.members;
	for (unsigned id = 0; id < RC_MAX; id++) {
		rc.t[id].direct_permille = seed % 17 == 2 && id == seed % RC_MAX ? 0 : 1000;
		rc.maintenance_ms[id] = seed % 13 == 0 ? 1000 : seed % 13 == 1 ? 3500 : 5000;
		rc.report_ms[id] = seed % 7 == 0 ? 0 : 5000;
	}
	/* Full, backup-only, newest-backup-only, sparse, common and distinct helpers. */
	unsigned mode = seed / 1000 % 6;
	for (unsigned age = 0; age < relay_control_cache_frames(&c); age++) {
		unsigned i = (c.frame - age) % relay_control_cache_frames(&c);
		rc.watch[i].frame = c.frame - age;
		rc.watch[i].members = c.members;
		rc.watch[i].survey = seed % 3 == 0 ? c.members : 0;
		rc.watch[i].primary_valid = mode == 1 || mode == 2 ? 0 :
			mode == 3 ? rnd() & c.members : c.members;
		rc.watch[i].backup_valid = mode == 2 && age != 0 ? 0 :
			mode == 3 ? rnd() & c.members : c.members;
		for (unsigned id = 0; id < RC_MAX; id++) {
			rc.watch[i].helpers[id] = seed < 4 || mode == 4 ? 0x11 : mode == 5 ? id * 0x11 : rnd();
			if (seed % 4 == 0) {
				rc.watch[i].rescue_tried[id]
					= (1U << (rc.watch[i].helpers[id] & 15U)) | (1U << (rc.watch[i].helpers[id] >> 4));
			}
		}
	}
	relay_coord_plan_grants(&rc, &c, &plan, 5000);
	relay_control_encode(payload, &c);
	crc = relay_control_plan_crc(payload);
	for (unsigned age = 0; age < relay_control_cache_frames(&c); age++) {
		unsigned i = plan.receipt_index[age];
		rc.receipt[i].frame = c.frame - age;
		rc.receipt[i].received = ~(seed % 5 == 0 ? 0 : seed % 5 == 1 ? c.members : rnd() & c.members);
	}
}

#if defined(__GNUC__) && !defined(__clang__)
__attribute__((optimize("O3")))
#endif
void timed(void)
{
	for (unsigned age = 0; age < relay_control_cache_frames(&c); age++) {
		const struct rc_receipt *r = &rc.receipt[plan.receipt_index[age]];
		uint16_t got = r->frame == c.frame - age ? r->received : 0;
		relay_control_missing(&c, age) = c.expected & ~got;
	}
	if (c.on_demand) {
		uint16_t debt = 0;
		for (unsigned age = 0; age < 5; age++) {
			debt |= relay_control_missing(&c, age) & plan.demand_eligible[age];
		}
		relay_demand_update(&demand_state, c.frame, relay_mask_count(debt), &demand_settings);
		relay_control_set_next2_repairs(&c, demand_state.queued[2]);
	}
	relay_coord_finish_frozen(&c, &plan, payload);
	relay_control_finish_encode(payload, &c, crc, false);
}

#ifdef RELAY_BENCH_FREESTANDING
void *memset(void *dst, int value, size_t n)
{
	unsigned char *p = dst;
	while (n--) *p++ = value;
	return dst;
}

void *memcpy(void *dst, const void *src, size_t n)
{
	unsigned char *p = dst;
	const unsigned char *q = src;
	while (n--) *p++ = *q++;
	return dst;
}
#endif
