/* Freeze equal-R coverage against the unchanged static allocator. */
#include "relay_coord.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static struct relay_grant observed[2][5];
static unsigned example(bool demand, unsigned repairs, bool late)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	rc.cache_frames = demand ? 5 : 3;
	rc.learn_members = 0x7ff;
	for (unsigned h = 0; h < RC_MAX; h++) {
		rc.t[h].direct_permille = 1000;
		rc.maintenance_ms[h] = rc.report_ms[h] = 1000;
	}
	struct relay_control c
		= {.on_demand = demand,
		   .members = 0x7ff,
		   .expected = 0x7ff,
		   .frame = 12,
		   .session = 42,
		   .epoch = 2,
		   .repair_count = repairs,
		   .primary_valid = 0x7ff,
		   .backup_valid = 0x7ff};
	for (unsigned ch = 0; ch < 11; ch++) {
		c.helpers[ch] = ((ch + 3) % 11) | (((ch + 6) % 11) << 4);
	}
	for (unsigned age = 0; age < rc.cache_frames; age++) {
		unsigned i = (c.frame - age) % rc.cache_frames;
		rc.watch[i] = (struct rc_watch){
			.frame = c.frame - age,
			.session = 42,
			.epoch = 2,
			.members = 0x7ff,
			.primary_valid = 0x7ff,
			.backup_valid = 0x7ff
		};
		memcpy(rc.watch[i].helpers, c.helpers, sizeof(c.helpers));
		rc.receipt[i] = (struct rc_receipt){
			.frame = c.frame - age,
			.received = age  ? 0x7ff
					  : late ? 0
							 : 0x7ff & ~(1U << 9)
		};
	}
	struct rc_grant_plan plan;
	relay_coord_plan_grants(&rc, &c, &plan, 1000);
	uint8_t wire[RELAY_CONTROL_LEN];
	relay_control_encode(wire, &c);
	/* Receipts only increase: early current frame has none, freeze has all
	 * except child 9. Older frames were already delivered in both snapshots. */
	c.missing[0] = 1U << 9;
	relay_coord_finish_frozen(&c, &plan, wire);
	relay_coord_read_grants(&c, wire);
	memcpy(observed[demand], c.grants, sizeof(c.grants));
	unsigned count = 0;
	printf("%s R%u %s age-zero miss child9:", demand ? "demand" : "static", repairs, late ? "late" : "early");
	for (unsigned s = 0; s < c.repair_count; s++) {
		printf(" slot%u=%u/0x%04x", s, c.grants[s].owner, c.grants[s].children);
		count += !!(c.grants[s].children & (1U << 9));
	}
	printf("; child grants=%u\n", count);
	return count;
}
int main(void)
{
	setbuf(stdout, NULL);
	for (unsigned repairs = 1; repairs <= 5; repairs++) {
		for (unsigned late = 0; late < 2; late++) {
			unsigned a = example(false, repairs, late), b = example(true, repairs, late);
			assert(a == (repairs < 3 ? repairs : 3) && a == b);
			for (unsigned i = 0; i < repairs; i++) {
				assert(observed[0][i].owner == observed[1][i].owner);
				assert(observed[0][i].children == observed[1][i].children);
			}
		}
	}
	puts("PASS equal R1-R5: demand freeze reserves the same primary, backup and retry as static");
	struct relay_coord rc;
	relay_coord_init(&rc);
	assert(relay_coord_publish(&rc, 3, 138930));
	assert(!relay_coord_publish(&rc, 3, 138929));
	assert(!relay_coord_was_published(&rc, 3, 138929));
	puts(
		"REPRODUCED receive/publication distinction: an age-one original received after frame138930 cannot be published"
	);
}
