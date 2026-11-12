/* Host convenience orchestration of production split APIs. */
#ifndef GRANTS_SUPPORT_H
#define GRANTS_SUPPORT_H
#include "relay_coord.h"
static inline void relay_coord_finish_grants(struct relay_coord *rc, struct relay_control *c,
	const struct rc_grant_plan *p)
{
	/* Receipts remain live until the freeze, including repaired old frames. */
	for (unsigned age = 0; age < RELAY_V3_CACHE_FRAMES; age++) {
		const struct rc_receipt *r = &rc->receipt[p->receipt_index[age]];
		if ((p->valid & (1U << age)) && r->frame == p->watch[age].frame) {
			c->missing[age] &= ~r->received;
		}
	}

	uint8_t wire[RELAY_CONTROL_LEN];
	relay_control_encode(wire, c);
	relay_coord_finish_frozen(c, p, wire);
	relay_coord_read_grants(c, wire);
}
static inline void relay_coord_build_grants(struct relay_coord *rc, struct relay_control *c, uint32_t now_ms)
{
	struct rc_grant_plan plan;
	relay_coord_plan_grants(rc, c, &plan, now_ms);
	relay_coord_finish_grants(rc, c, &plan);
	relay_coord_commit_grants(rc, c, &plan);
}
#endif
