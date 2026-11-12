#include "grants_support.h"
#include <assert.h>
#include <stdio.h>
#include "relay_coord.h"

static void spare_repair_retry(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	struct relay_control c = {.members = (1U << 9) | (1U << 7) | 1U,
		.frame = 12, .repair_count = 5, .missing = {1U << 9}};
	for (unsigned i = 0; i < RC_MAX; i++) {
		rc.maintenance_ms[i] = rc.report_ms[i] = 1000;
	}
	rc.report_ms[0] = 0;
	rc.watch[0] = (struct rc_watch){.frame = 12, .primary_valid = 1U << 9};
	rc.watch[0].helpers[9] = 7;
	relay_coord_build_grants(&rc, &c, 1000);
	assert(c.grants[0].owner == 7 && c.grants[0].children == (1U << 9));
	assert(c.grants[1].owner == 0 && !c.grants[1].children); /* report first */
	assert(c.grants[2].owner == 7 && c.grants[2].children == (1U << 9));
	assert(c.grants[3].owner == RELAY_OWNER_IDLE);
	/* No spare capacity: the report is never displaced by a retry. */
	c.repair_count = 2;
	rc.report_ms[0] = 0;
	relay_coord_build_grants(&rc, &c, 1000);
	assert(c.grants[0].owner == 7 && c.grants[0].children == (1U << 9));
	assert(c.grants[1].owner == 0 && !c.grants[1].children);
}

int main(void)
{
	spare_repair_retry();
	struct relay_coord rc;
	relay_coord_init(&rc);
	assert(!rc.v3_requested && !relay_coord_v3_ready(&rc, 1));
	for (unsigned id = 0; id < RC_MAX; id++) {
		relay_coord_note_caps(&rc, id, RELAY_CAP_HELPER | (RELAY_V3_VERSION << RELAY_CAP_VERSION_SHIFT));
		assert(!relay_coord_v3_ready(&rc, (uint16_t)(1U << id)));
	}
	rc.v3_requested = true;
	for (unsigned id = 0; id < RC_MAX; id++) {
		assert(relay_coord_v3_ready(&rc, (uint16_t)(1U << id)));
	}
	assert(relay_coord_v3_ready(&rc, 0x83FF));
	assert(relay_coord_v3_ready(&rc, 0xFFFF) && !relay_coord_v3_ready(&rc, 0));
	relay_coord_note_caps(&rc, 15, RELAY_CAP_HELPER | (RELAY_PROTOCOL_VERSION << RELAY_CAP_VERSION_SHIFT));
	assert(rc.t[15].capable && !relay_coord_v3_ready(&rc, 0x8001));
	relay_coord_note_caps(&rc, 15, 0xF1);
	assert(!rc.t[15].capable && !relay_coord_v3_ready(&rc, 0x8001));
	rc.enabled = false;
	assert(!relay_coord_v3_ready(&rc, 1));

	rc.enabled = true;
	relay_coord_fast_reset(&rc);
	for (unsigned id = 0; id < RC_MAX; id++)
		{ rc.t[id].rssi = 40 + id; rc.t[id].direct_permille = 1000; }
	struct relay_control c = {.members = 0x7ff, .expected = 0x7ff, .repair_count = 5, .frame = 0};
	rc.learn_members = c.members;
	relay_coord_learn_tick(&rc, 0);
	relay_coord_plan_watch(&rc, &c);
	relay_coord_build_grants(&rc, &c, 0);
	unsigned load[RC_MAX] = {0};
	for (unsigned id = 0; id < 11; id++) {
		for (unsigned copy = 0; copy < 2; copy++) {
			if (!((copy ? c.backup_valid : c.primary_valid) & (1U << id)))
				continue;
			unsigned helper = (c.helpers[id] >> (copy * 4U)) & 15U;
			assert(helper + 1 < id || id + 1 < helper);
			assert(++load[helper] <= RELAY_MAX_CHILDREN);
		}
	}
	c.frame = 1;
	c.missing[0] = 1;
	relay_coord_plan_watch(&rc, &c);
	relay_coord_build_grants(&rc, &c, RC_MAINTENANCE_MS);
	assert(c.grants[0].children & 1);
	assert(c.grants[1].children & 1); /* primary silence leaves a backup grant */
	assert(c.grants[0].owner != c.grants[1].owner);
	assert(!(c.flags & RELAY_CONTROL_MAINTENANCE)); /* due, but repairs hold R0-R1 */
	/* Idle R0-R1 go to due members in rotation. Frame 10 leaves frame 1's watch map alone. */
	struct relay_control m = {.members = 7, .repair_count = 5, .frame = 10};
	relay_coord_build_grants(&rc, &m, RC_MAINTENANCE_MS - 1);
	assert(!(m.flags & RELAY_CONTROL_MAINTENANCE) && m.maintenance_owner == RELAY_OWNER_IDLE);
	for (unsigned owner = 0; owner < 3; owner++) {
		relay_coord_build_grants(&rc, &m, RC_MAINTENANCE_MS);
		assert((m.flags & RELAY_CONTROL_MAINTENANCE) && m.maintenance_owner == owner);
		assert(m.grants[0].owner == RELAY_OWNER_MAINTENANCE && !m.grants[0].children);
		assert(m.grants[1].owner == RELAY_OWNER_MAINTENANCE && !m.grants[1].children);
	}
	/* Equal ages start at the rotating id. */
	rc.maintenance_next = 1;
	relay_coord_build_grants(&rc, &m, 2 * RC_MAINTENANCE_MS);
	assert(m.maintenance_owner == 1);
	/* Repairs hold R0-R1 until a member waits RC_MAINTENANCE_URGENT_MS,
	 * then it takes them and repairs move to R2-R3. */
	rc.maintenance_ms[2] = 0;
	m.frame = c.frame; m.missing[0] = 1;
	relay_coord_build_grants(&rc, &m, RC_MAINTENANCE_URGENT_MS - 1);
	assert(!(m.flags & RELAY_CONTROL_MAINTENANCE) && (m.grants[0].children & 1));
	m.missing[0] = 1;
	relay_coord_build_grants(&rc, &m, RC_MAINTENANCE_URGENT_MS);
	assert(m.maintenance_owner == 2);
	assert(m.grants[0].owner == RELAY_OWNER_MAINTENANCE && !m.grants[0].children);
	assert(m.grants[1].owner == RELAY_OWNER_MAINTENANCE && !m.grants[1].children);
	assert(m.grants[2].children & 1);
	assert(m.grants[3].children & 1);
	/* RF receipt before the event worker must suppress B repair demand,
	 * without suppressing the first publication or allowing duplicates. */
	relay_coord_receive(&rc, 0, 1);
	assert(rc.receipt[1].frame == 1 && (rc.receipt[1].received & 1));
	c.missing[0] = 1;
	relay_coord_build_grants(&rc, &c, 1);
	assert(!(c.missing[0] & 1));
	assert(relay_coord_publish(&rc, 0, 1));
	assert(!relay_coord_publish(&rc, 0, 1));
	assert(relay_coord_publish(&rc, 0, 2));
	assert(!relay_coord_publish(&rc, 0, 0));
	assert(relay_coord_was_published(&rc, 0, 1));
	assert(!relay_coord_was_published(&rc, 0, 0));
	assert(relay_coord_publish(&rc, 0, 4));
	assert(!relay_coord_publish(&rc, 0, 3));
	assert(!relay_coord_was_published(&rc, 0, 3));
	assert(relay_coord_was_published(&rc, 0, 2));
	relay_coord_fast_reset(&rc);
	assert(relay_coord_publish(&rc, 0, UINT32_MAX));
	assert(relay_coord_publish(&rc, 0, 0));
	assert(relay_coord_was_published(&rc, 0, UINT32_MAX));
	puts("v3 negotiation, watch capacity, primary/backup grants and frame ledger: passed");
	return 0;
}
