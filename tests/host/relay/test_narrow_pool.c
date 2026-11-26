#include "grants_support.h"
#include <assert.h>
#include <stdio.h>
#include "relay_coord.h"
static struct relay_coord rc;
static struct relay_control c;
static struct rc_grant_plan p;
static void setup(unsigned frame)
{
	relay_coord_init(&rc);
	c = (struct relay_control){
		.members = 0x7ff,
		.expected = (1U << 3) | (1U << 9),
		.frame = frame,
		.session = 42,
		.epoch = 3,
		.repair_count = 3,
		.profile = 1
	};
	rc.learn_members = c.members;
	for (unsigned h = 0; h < RC_MAX; h++) {
		rc.maintenance_ms[h] = rc.report_ms[h] = 5000;
	}
	rc.t[0].direct_permille = 1000;
	for (unsigned age = 0; age < 3; age++) {
		unsigned f = frame - age, i = f % 3;
		rc.watch[i] = (struct rc_watch){
			.frame = f,
			.session = 42,
			.epoch = 3,
			.members = c.members,
			.survey = c.expected,
			.primary_valid = c.expected,
			.backup_valid = c.expected
		};
		rc.watch[i].helpers[3] = rc.watch[i].helpers[9] = 0x75;
		rc.receipt[i] = (struct rc_receipt){.frame = f};
		c.missing[age] = c.expected;
	}
}
int main(void)
{
	for (unsigned frame = 100; frame < 110; frame++) {
		setup(frame);
		relay_coord_plan_grants(&rc, &c, &p, 5000);
		assert(p.rescue_child == (1U << (frame % 2 ? 9 : 3)));
	}
	setup(101);
	rc.t[2].direct_permille = 1000;
	rc.peer[0][9] = (struct rc_peer){.expected = 100, .heard = 96, .rssi_x8 = 20 * 8};
	rc.peer[2][9] = (struct rc_peer){.expected = 100, .heard = 99, .rssi_x8 = 25 * 8};
	relay_coord_plan_grants(&rc, &c, &p, 5000);
	assert(p.rescue_helper == 2);
	rc.t[2].direct_permille = 949;
	relay_coord_plan_grants(&rc, &c, &p, 5000);
	assert(p.rescue_helper == 0);
	struct relay_grant original[3];
	for (unsigned n = 0; n < 3; n++) {
		setup(100 + 2 * n);
		relay_coord_plan_grants(&rc, &c, &p, 5000);
		p.emergency_child = 0;
		relay_coord_finish_grants(&rc, &c, &p);
		for (unsigned i = 0; i < 3; i++) {
			unsigned at = (i + c.frame % 3) % 3;
			if (!n) {
				original[at] = c.grants[i];
			} else {
				assert(original[at].owner == c.grants[i].owner && original[at].children == c.grants[i].children);
			}
		}
	}
	for (unsigned frame = 100; frame < 103; frame++) {
		setup(frame);
		rc.maintenance_ms[0] = 0;
		relay_coord_plan_grants(&rc, &c, &p, 5000);
		relay_coord_finish_grants(&rc, &c, &p);
		assert(
			(c.flags & RELAY_CONTROL_MAINTENANCE) && c.grants[0].owner == RELAY_OWNER_MAINTENANCE
			&& c.grants[1].owner == RELAY_OWNER_MAINTENANCE
		);
	}
	setup(101);
	rc.t[7].direct_permille = 1000;
	rc.watch[(c.frame - 2) % 3].r0_only[9] = 1U << 7;
	rc.maintenance_ms[0] = 0;
	relay_coord_plan_grants(&rc, &c, &p, 5000);
	assert(p.emergency_child == (1U << 9) && p.emergency_helper == 7);
	relay_coord_finish_grants(&rc, &c, &p);
	assert(c.grants[0].owner == RELAY_OWNER_MAINTENANCE && c.grants[1].owner == RELAY_OWNER_MAINTENANCE);
	assert(c.grants[2].owner == 7 && c.grants[2].children == c.expected);
	setup(101);
	relay_coord_plan_grants(&rc, &c, &p, 5000);
	for (unsigned age = 0; age < 3; age++) {
		rc.receipt[(c.frame - age) % 3].received = c.expected;
	}
	relay_coord_finish_grants(&rc, &c, &p);
	for (unsigned i = 0; i < 3; i++) {
		assert(!c.grants[i].children);
	}
	setup(101);
	for (unsigned h = 5; h <= 7; h++) {
		rc.t[h].direct_permille = 1000;
	}
	for (unsigned age = 0; age < 3; age++) {
		rc.watch[(c.frame - age) % 3].helpers[3] = 0x56;
		rc.receipt[(c.frame - age) % 3].received = age == 2 ? (1U << 3) : (1U << 9);
	}
	relay_coord_plan_grants(&rc, &c, &p, 5000);
	relay_coord_finish_grants(&rc, &c, &p);
	assert(c.grants[0].owner == 5 && c.grants[1].owner == 7);
	assert((c.grants[0].children & (1U << 9)) && (c.grants[1].children & (1U << 9)));
	puts("Narrow pool: tied rescue fairness, physical slot rotation and fixed maintenance passed");
}
