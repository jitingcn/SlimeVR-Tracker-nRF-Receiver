/* Exercise the published watch ring and the production grant allocator. */
#include "grants_support.h"
#include <assert.h>
#include <stdio.h>

#define MASK(id) (1U << (id))
enum { CHILD = 9, FIRST_HELPER = 5, NEXT_HELPER = 7, LAST_SENT = 209, NOW = 5000 };
static bool on_demand;
static uint8_t repairs;
static struct relay_coord rc;
static struct rc_grant_plan plan;

static struct relay_control control(uint32_t frame, uint8_t helper)
{
	struct relay_control c = {
		.frame = frame, .session = 42, .epoch = 3, .profile = 1,
		.members = 0x7ff, .expected = MASK(CHILD), .repair_count = repairs, .on_demand = on_demand,
	};
	if (helper < RC_MAX) {
		c.primary_valid = MASK(CHILD);
		c.helpers[CHILD] = helper;
	}
	return c;
}

static void prepare(struct relay_control *c, uint32_t missing)
{
	for (unsigned age = 0; age < rc.cache_frames; age++) {
		uint32_t frame = c->frame - age;
		relay_control_missing(c, age) = frame == missing ? MASK(CHILD) : 0;
		if (frame != missing) {
			relay_coord_receive(&rc, CHILD, frame);
		}
	}
	relay_coord_fill_watch_gaps(&rc, c);
	rc.learn_members = c->members;
	relay_coord_plan_grants(&rc, c, &plan, NOW);
	relay_coord_finish_grants(&rc, c, &plan);
}

static unsigned owners(const struct relay_control *c)
{
	unsigned mask = 0;
	for (unsigned slot = 0; slot < c->repair_count; slot++) {
		const struct relay_grant *g = &c->grants[slot];
		if (g->owner < RC_MAX && (g->children & MASK(CHILD))) {
			mask |= MASK(g->owner);
		}
	}
	return mask;
}

static void reset(uint8_t helper)
{
	relay_coord_init(&rc);
	rc.cache_frames = on_demand ? RELAY_CACHE_MAX_FRAMES : RELAY_CACHE_FRAMES;
	for (unsigned id = 0; id < RC_MAX; id++) {
		rc.t[id].direct_permille = 1000;
		rc.maintenance_ms[id] = rc.report_ms[id] = NOW;
	}
	struct relay_control c = control(LAST_SENT, helper);
	prepare(&c, UINT32_MAX);
	relay_coord_commit_grants(&rc, &c, &plan);
}

static void test_unsent_and_superseding(void)
{
	reset(FIRST_HELPER);
	struct relay_control c = control(LAST_SENT + 1, NEXT_HELPER);
	prepare(&c, c.frame);
	assert(owners(&c) == MASK(FIRST_HELPER));
	assert(plan.survey_next != rc.survey_next);
	assert(!rc.watch[c.frame % rc.cache_frames].rescue_tried[CHILD]);
	/* Do not commit: this control never made it onto the air. */
	c = control(LAST_SENT + 2, NEXT_HELPER);
	prepare(&c, c.frame);
	assert(owners(&c) == MASK(FIRST_HELPER));
	assert(rc.survey_next == 0);
	assert(plan.watch[0].carried && !plan.watch[0].rescue_tried[CHILD]);
	relay_coord_commit_grants(&rc, &c, &plan);
	assert(rc.watch[c.frame % rc.cache_frames].rescue_tried[CHILD] == MASK(FIRST_HELPER));

	c = control(c.frame + 1, NEXT_HELPER);
	prepare(&c, c.frame);
	assert(owners(&c) & MASK(NEXT_HELPER));
	assert(!plan.watch[0].carried && plan.watch[0].helpers[CHILD] == NEXT_HELPER);
}

static void test_skipped_and_expired(void)
{
	reset(FIRST_HELPER);
	struct relay_control c = control(LAST_SENT + 3, NEXT_HELPER);
	prepare(&c, c.frame);
	assert(owners(&c) == MASK(FIRST_HELPER));
	assert(plan.watch[0].carried && plan.watch[1].carried);
	c = control(c.frame + 1, NEXT_HELPER);
	prepare(&c, c.frame);
	assert(!owners(&c));
	assert(!(plan.valid & 1U));

	/* The last in-horizon capture remains repairable for its full cache age. */
	reset(FIRST_HELPER);
	c = control(LAST_SENT + 3 + rc.cache_frames - 1U, NEXT_HELPER);
	prepare(&c, LAST_SENT + 3);
	assert(owners(&c) == MASK(FIRST_HELPER));
}

static void test_context_change(void)
{
	for (unsigned fault = 0; fault < 3; fault++) {
		reset(FIRST_HELPER);
		struct relay_control c = control(LAST_SENT + 2, NEXT_HELPER);
		prepare(&c, c.frame);
		assert(owners(&c) == MASK(FIRST_HELPER));
		/* Also reject a held watch already materialized in the old context. */
		if (fault == 0) c.session++;
		if (fault == 1) c.epoch++;
		if (fault == 2) c.members ^= MASK(0);
		prepare(&c, c.frame);
		assert(!owners(&c));
		assert(!(plan.valid & 1U));
	}
}

static void test_committed_survey(void)
{
	for (unsigned sent = 0; sent < 2; sent++) {
		/* No primary or backup ever covered the child: only the survey can
		 * explain a held copy from the sole qualified, nonadjacent helper. */
		reset(RC_MAX);
		rc.survey_next = CHILD;
		struct relay_control c = control(LAST_SENT + 1, RC_MAX);
		prepare(&c, UINT32_MAX);
		assert(c.survey == MASK(CHILD) && !c.primary_valid && !c.backup_valid);
		if (sent) {
			relay_coord_commit_grants(&rc, &c, &plan);
		}
		for (unsigned id = 0; id < RC_MAX; id++) {
			if (id != FIRST_HELPER && id != CHILD) rc.t[id].direct_permille = 0;
		}
		c = control(LAST_SENT + 3, NEXT_HELPER);
		assert(relay_can_overhear(c.members, FIRST_HELPER, CHILD));
		prepare(&c, c.frame);
		assert(plan.watch[0].carried && !plan.watch[0].survey);
		assert(plan.watch[0].listeners[CHILD] == (sent ? c.members : 0U));
		assert(owners(&c) == (sent ? MASK(FIRST_HELPER) : 0U));
		if (sent) {
			assert(plan.rescue_child == MASK(CHILD) && plan.rescue_age == 0);
			assert(plan.rescue_helper == FIRST_HELPER);
		}
	}
}

int main(void)
{
	for (unsigned mode = 0; mode < 2; mode++) {
		on_demand = mode != 0;
		for (repairs = 3; repairs <= 5; repairs += 2) {
			test_unsent_and_superseding();
			test_skipped_and_expired();
			test_context_change();
			test_committed_survey();
		}
	}
	puts("Published watch roles survive bounded control gaps in narrow, wide and on-demand repairs");
}
