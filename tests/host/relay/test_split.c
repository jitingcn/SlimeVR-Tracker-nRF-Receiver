#include "grants_support.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "relay_coord.h"
#define RC_BIT(id) (1U << (id))
#include "grants_reference.inc"
#ifdef COMPARE_CEDEEA0
void cedeea0_relay_coord_plan_grants(struct relay_coord *, struct relay_control *, struct rc_grant_plan *, uint32_t);
void cedeea0_relay_coord_finish_frozen(struct relay_control *, const struct rc_grant_plan *, uint8_t *);
void cedeea0_relay_coord_commit_grants(
	struct relay_coord *,
	const struct relay_control *,
	const struct rc_grant_plan *
);
#endif
static uint32_t seed = 0x1d34f921;
static uint32_t random_word(void)
{
 seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
 return seed;
}
static uint8_t bitwise_crc(uint8_t v, const uint8_t *p, size_t n)
{
 while (n--) {
  v ^= *p++;
  for (unsigned b = 0; b < 8; b++) v = (uint8_t)((v << 1) ^ ((v & 128) ? 7 : 0));
 }
 return v;
}
static void check_math(uint32_t origin, uint32_t frame, uint32_t tick)
{
	assert(
		relay_frame_start(origin, frame)
		== origin + (uint32_t)((uint64_t)frame * RELAY_V3_GRID_TICKS / RELAY_V3_GRID_FRAMES)
	);
	assert(
		relay_frame_at_tick(origin, tick)
		== (uint32_t)(((uint64_t)(uint32_t)(tick - origin) * RELAY_V3_GRID_FRAMES + RELAY_V3_GRID_FRAMES - 1U)
					  / RELAY_V3_GRID_TICKS)
	);
}
int main(void)
{
 const uint32_t edges[] = {0,1,14,15,16,4095,4096,4097,0x7fffffff,0x80000000,0xfffffffe,0xffffffff};
 for (unsigned i=0;i<sizeof(edges)/sizeof(*edges);i++)
  for (unsigned j=0;j<sizeof(edges)/sizeof(*edges);j++)
   for (unsigned k=0;k<sizeof(edges)/sizeof(*edges);k++) check_math(edges[i],edges[j],edges[k]);
 for (unsigned i=0;i<1000000;i++) {uint32_t o=random_word(), f=random_word(), t=random_word();check_math(o,f,t);}

 static struct relay_coord a, b;
 for (unsigned run = 0; run < 50000; run++) {
  uint8_t bytes[256];
  for (unsigned i = 0; i < sizeof(bytes); i++) bytes[i] = random_word();
  unsigned len = random_word() % 257; uint8_t init = random_word();
  assert(relay_crc8(init, bytes, len) == bitwise_crc(init, bytes, len));
  relay_coord_init(&a);
  uint32_t now = 5000 + random_word() % 10000;
  struct relay_control c = {.members = random_word(), .frame = 10 + run,
   .repair_count = RELAY_V3_REPAIR_SLOTS, .profile = 1};
  c.expected = c.members;
  c.primary_valid = random_word() & c.members;
  c.backup_valid = random_word() & c.members;
  a.learn_members = c.members;
  a.maintenance_next = random_word() % RC_MAX;
  a.survey_next = random_word() % RC_MAX;
  a.report_next = random_word() % RC_MAX;
  for (unsigned id = 0; id < RC_MAX; id++) {
   a.t[id].direct_permille = random_word() % 2 ? 1000 : 949;
   a.maintenance_ms[id] = now - random_word() % 4001;
   a.report_ms[id] = now - random_word() % 1501;
   c.helpers[id] = random_word();
   a.t[id].rssi = random_word() % 80;
   for (unsigned h = 0; h < RC_MAX; h++) {
	   a.peer[h][id]
		   = (struct rc_peer){.expected = 100, .heard = random_word() % 101, .rssi_x8 = 100 + random_word() % 700};
   }
  }
  for (unsigned age = 0; age < RELAY_V3_CACHE_FRAMES; age++) {
   unsigned i = (c.frame - age) % RELAY_V3_CACHE_FRAMES;
   a.watch[i] = (struct rc_watch){.frame = c.frame - age,
    .members = c.members, .survey = random_word() & c.members,
    .primary_valid = random_word() & c.members, .backup_valid = random_word() & c.members};
   if (run % 7 == 0) a.watch[i].frame = UINT32_MAX;
   for (unsigned id = 0; id < RC_MAX; id++) {
	   a.watch[i].helpers[id] = random_word();
	   a.watch[i].listeners[id] = random_word() & c.members;
	   a.watch[i].rescue_tried[id] = random_word() & c.members;
   }
   a.receipt[i] = (struct rc_receipt){.frame = c.frame - age, .received = random_word()};
  }
  b = a;
  struct relay_control d = c;
  struct rc_grant_plan plan;
  relay_coord_plan_grants(&b, &d, &plan, now);
#ifdef COMPARE_CEDEEA0
  struct rc_grant_plan old_plan;
  cedeea0_relay_coord_plan_grants(&a, &c, &old_plan, now);
  assert(memcmp(&old_plan, &plan, sizeof(plan)) == 0);
#endif
  uint8_t planned[RELAY_CONTROL_LEN];
  relay_control_encode(planned, &d);
  struct relay_control_crc_plan crc_plan = relay_control_plan_crc(planned);
  /* Missing evidence and receipts can arrive after planning. */
  for (unsigned age = 0; age < RELAY_V3_CACHE_FRAMES; age++) {
   c.missing[age] = d.missing[age] = random_word() & c.members;
   unsigned i = (c.frame - age) % RELAY_V3_CACHE_FRAMES;
   a.receipt[i].received = b.receipt[i].received = random_word();
  }
#ifdef COMPARE_CEDEEA0
  for (unsigned age = 0; age < RELAY_V3_CACHE_FRAMES; age++) {
	  const struct rc_receipt *r = &a.receipt[old_plan.receipt_index[age]];
	  if ((old_plan.valid & (1U << age)) && r->frame == old_plan.watch[age].frame) {
		  c.missing[age] &= ~r->received;
	  }
  }
  uint8_t old_wire[RELAY_CONTROL_LEN];
  relay_control_encode(old_wire, &c);
  cedeea0_relay_coord_finish_frozen(&c, &old_plan, old_wire);
  relay_coord_read_grants(&c, old_wire);
  cedeea0_relay_coord_commit_grants(&a, &c, &old_plan);
#else
  reference_build_grants(&a, &c, now, &plan);
#endif
  relay_coord_finish_grants(&b, &d, &plan);
  relay_coord_commit_grants(&b, &d, &plan);
  uint8_t x[RELAY_CONTROL_LEN], y[RELAY_CONTROL_LEN];
  relay_control_encode(x, &c); relay_control_encode(y, &d);
  assert(memcmp(x, y, sizeof(x)) == 0);
  relay_control_finish_encode(planned, &d, crc_plan, true);
  assert(memcmp(x, planned, sizeof(x)) == 0);
  assert(memcmp(&a, &b, sizeof(a)) == 0);
 }
 puts("50000 split grant/state and random CRC comparisons passed");
}
