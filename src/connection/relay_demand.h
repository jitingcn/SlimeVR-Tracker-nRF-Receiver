#ifndef RELAY_DEMAND_H
#define RELAY_DEMAND_H
#include "relay_protocol.h"
#define RELAY_DEMAND_HOLD 30U
/* Fixed cost for the frozen demand path, independent of burst cardinality. */
static inline __attribute__((always_inline)) unsigned relay_demand_count(uint16_t mask)
{
	uint32_t n = mask;
	n -= (n >> 1) & 0x5555U;
	n = (n & 0x3333U) + ((n >> 2) & 0x3333U);
	n = (n + (n >> 4)) & 0x0f0fU;
	return (n + (n >> 8)) & 0x1fU;
}
struct relay_demand {
	uint32_t frame, quiet;
	uint8_t queued[3], target;
	bool valid;
};
static inline __attribute__((always_inline)) uint8_t relay_demand_slots(uint32_t frame, uint8_t target)
{
	return target < 2U && frame % 8U == 0 ? 2U : target;
}
static inline __attribute__((always_inline)) void
relay_demand_reset(struct relay_demand *d, uint32_t frame, uint8_t floor)
{
	*d = (struct relay_demand){.frame = frame, .target = floor, .valid = true};
	for (unsigned i = 0; i < 3; i++) {
		d->queued[i] = relay_demand_slots(frame + i, floor);
	}
}
static inline __attribute__((always_inline)) void
relay_demand_advance(struct relay_demand *d, uint32_t frame, const struct relay_runtime_settings *s)
{
	if (!d->valid) {
		relay_demand_reset(d, frame, s->repair_floor);
	}
	while (d->frame != frame) {
		d->frame++;
		d->queued[0] = d->queued[1];
		d->queued[1] = d->queued[2];
		d->queued[2] = relay_demand_slots(d->frame + 2U, d->target);
	}
}
/* Only f+2 can change. The caller chooses once per B before wire encoding. */
static inline __attribute__((always_inline)) void
relay_demand_update(struct relay_demand *d, uint32_t frame, unsigned missing, const struct relay_runtime_settings *s)
{
	relay_demand_advance(d, frame, s);
	if (missing) {
		d->quiet = 0;
		unsigned target = missing > s->max_repairs ? s->max_repairs : missing;
		if (target > d->target) {
			d->target = target;
		}
	} else if (++d->quiet >= RELAY_DEMAND_HOLD) {
		if (d->target > s->repair_floor) {
			d->target--;
		}
		d->quiet = 0;
	}
	if (d->target < s->repair_floor) {
		d->target = s->repair_floor;
	}
	d->queued[2] = relay_demand_slots(frame + 2U, d->target);
}
#endif
