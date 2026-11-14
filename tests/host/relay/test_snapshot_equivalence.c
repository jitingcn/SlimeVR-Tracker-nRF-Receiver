/* Compare compact inputs with the full coordinator implementation at b4dac52. */
#include "relay_coord.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
void baseline_relay_coord_learn_tick(struct relay_coord *, uint32_t);
void baseline_relay_coord_plan_watch(struct relay_coord *, struct relay_control *);
void baseline_relay_coord_plan_grants(struct relay_coord *, struct relay_control *, struct rc_grant_plan *, uint32_t);
static uint32_t seed = 0x147951;
static uint32_t next(void)
{
	seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
	return seed;
}
int main(void)
{
	for (unsigned run = 0; run < 4000; run++) {
		struct relay_coord a = {0}, b;
		uint32_t now = next();
		a.cache_frames = run % 2 ? 5 : 3;
		a.learn_members = next();
		for (unsigned c = 0; c < RC_MAX; c++) {
			a.t[c].direct_permille = next() % 1001;
			a.t[c].rssi = next() % 100;
			a.maintenance_ms[c] = now - next() % 4001;
			a.report_ms[c] = now - next() % 2001;
			for (unsigned h = 0; h < RC_MAX; h++)
				a.peer[h][c] = (struct rc_peer){.expected = 120, .heard = next() % 121,
					.rssi_x8 = next() % 800, .evidence_ms = now - next() % 12001};
			for (unsigned k = 0; k < 2; k++)
				a.choice[c][k] = (struct rc_choice){.valid = next() % 2, .helper = next() % RC_MAX,
					.challenger = next() % RC_MAX, .streak = next() % 3, .since_ms = now - next() % 2001};
		}
		b = a;
		relay_coord_learn_tick(&a, now);
		baseline_relay_coord_learn_tick(&b, now);
		assert(!memcmp(&a, &b, sizeof(a)));
		struct relay_control c = {.members = a.learn_members, .expected = next(), .frame = next(),
			.session = 42, .epoch = 7, .profile = 1, .repair_count = 1 + next() % 5,
			.on_demand = a.cache_frames == 5};
		for (unsigned age = 0; age < a.cache_frames; age++) {
			unsigned i = (c.frame - age) % a.cache_frames;
			a.watch[i] = (struct rc_watch){.frame = c.frame - age, .session = c.session,
				.epoch = c.epoch, .members = c.members, .survey = next(),
				.primary_valid = next(), .backup_valid = next()};
			for (unsigned ch = 0; ch < RC_MAX; ch++) {
				a.watch[i].helpers[ch] = next();
				a.watch[i].listeners[ch] = next();
				a.watch[i].rescue_tried[ch] = next();
			}
			a.receipt[i] = (struct rc_receipt){.frame = c.frame - age, .received = next()};
		}
		b = a;
		struct relay_control d = c;
		relay_coord_plan_watch(&a, &c);
		baseline_relay_coord_plan_watch(&b, &d);
		assert(!memcmp(&c, &d, sizeof(c)));
		struct rc_grant_plan p, q;
		relay_coord_plan_grants(&a, &c, &p, now);
		baseline_relay_coord_plan_grants(&b, &d, &q, now);
		assert(!memcmp(&p, &q, sizeof(p)));
		assert(!memcmp(&c, &d, sizeof(c)));
		assert(!memcmp(&a, &b, sizeof(a)));
	}
	puts("4000 full/compact learning, watch and grant comparisons passed, static ages 0-2 and demand ages 0-4");
}
