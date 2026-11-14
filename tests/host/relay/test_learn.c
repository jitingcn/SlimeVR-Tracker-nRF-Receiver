#include "grants_support.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "relay_coord.h"

static void evidence(struct relay_coord *rc, unsigned h, unsigned child,
	unsigned expected, unsigned heard, unsigned rssi, unsigned now)
{
	struct relay_control c = {.members = 0xffff, .session = 42, .epoch = 7, .repair_count = 5};
	c.grants[2].owner = h;
	struct relay_peer_report r = {.helper = h, .session = 42, .epoch = 7, .count = 1};
	r.entries[0] = (struct relay_peer_observation){.child = child, .expected = expected,
		.heard = heard, .rssi = rssi};
	assert(relay_coord_report(rc, &c, 2, &r, now));
	c.grants[2].children = 1;
	assert(!relay_coord_report(rc, &c, 2, &r, now));
	c.grants[2].children = 0; c.grants[2].owner = h + 1;
	assert(!relay_coord_report(rc, &c, 2, &r, now));
	c.grants[2].owner = h; r.session++;
	assert(!relay_coord_report(rc, &c, 2, &r, now));
	r.session--; r.epoch++;
	assert(!relay_coord_report(rc, &c, 2, &r, now));
	assert(!relay_coord_report(rc, &c, 5, &r, now));
}

static void learning(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	rc.learn_members = 0x1f;
	for (unsigned h = 2; h <= 4; h++) {
		rc.t[h].direct_permille = 1000;
		rc.t[h].rssi = 30 + h;
	}
	evidence(&rc, 2, 0, 120, 60, 20, 0);
	evidence(&rc, 3, 0, 120, 114, 60, 0);
	evidence(&rc, 4, 0, 120, 114, 65, 0);
	assert(!rc.peer[0][3].expected); /* directed */
	relay_coord_learn_tick(&rc, 0);
	assert(rc.choice[0][0].helper == 3 && rc.choice[0][1].helper == 4);
	assert(relay_coord_peer_ratio(&rc.peer[3][0]) == 950);
	/* A tempting RSSI on a poor capture link never wins. */
	assert(rc.choice[0][0].helper != 2);
	evidence(&rc, 2, 1, 120, 0, 10, 0);
	assert(rc.peer[2][1].expected == 120 && !rc.peer[2][1].heard && !rc.peer[2][1].rssi_x8);
	relay_coord_capture(&rc, 2, 1, 55, 100);
	assert(rc.peer[2][1].expected == 120 && !rc.peer[2][1].heard);
	assert(rc.peer[2][1].rssi_x8 == 440);
	/* A sustained 50 permille gain replaces the primary on the second tick. */
	rc.peer[2][0] = (struct rc_peer){.expected=120,.heard=120,.rssi_x8=160,.evidence_ms=0};
	relay_coord_learn_tick(&rc, 1000);
	assert(rc.choice[0][0].helper == 3);
	rc.peer[2][0].heard = 114;
	rc.peer[2][0].rssi_x8 = 480; /* no margin resets the streak */
	relay_coord_learn_tick(&rc, 2000);
	assert(rc.choice[0][0].helper == 3);
	rc.peer[2][0].heard = 120; rc.peer[2][0].rssi_x8 = 472;
	relay_coord_learn_tick(&rc, 3000);
	assert(rc.choice[0][0].helper == 3);
	relay_coord_learn_tick(&rc, 4000);
	assert(rc.choice[0][0].helper == 2);
	/* A failing incumbent bypasses the hold and two-tick requirement. */
	rc.peer[2][0].heard = 90;
	relay_coord_learn_tick(&rc, 4500);
	assert(rc.choice[0][0].helper == 3);
	evidence(&rc, 3, 0, 120, 114, 60, 5000);
	assert(rc.peer[3][0].expected == 120 && rc.peer[3][0].heard == 114);
	relay_coord_learn_tick(&rc, 15000);
	assert(!rc.peer[3][0].expected && !rc.peer[3][0].rssi_x8);
	assert(rc.choice[0][0].valid); /* provisional fallback */
}

static void rssi_hold(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc); rc.learn_members=31;
	for (unsigned h=2;h<5;h++) rc.t[h].direct_permille=1000;
	evidence(&rc,2,0,120,120,50,0);
	evidence(&rc,3,0,120,120,60,0);
	evidence(&rc,4,0,120,120,70,0);
	relay_coord_learn_tick(&rc,0);
	assert(rc.choice[0][0].helper==2);
	rc.peer[4][0].rssi_x8=352; /* 6 dB stronger */
	relay_coord_learn_tick(&rc,500);
	assert(rc.choice[0][0].helper==2 && !rc.choice[0][0].streak);
	relay_coord_learn_tick(&rc,1000);
	assert(rc.choice[0][0].helper==2);
	relay_coord_learn_tick(&rc,2000);
	assert(rc.choice[0][0].helper==4);
	/* Small RSSI changes do not flap the pair. */
	rc.peer[2][0].rssi_x8=344;
	relay_coord_learn_tick(&rc,3000); relay_coord_learn_tick(&rc,4000);
	assert(rc.choice[0][0].helper==4);
}

static void coverage(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	struct relay_control c = {.members=0x7ff,.repair_count=5};
	rc.learn_members = c.members;
	relay_coord_learn_tick(&rc, 0);
	relay_coord_plan_watch(&rc, &c);
	assert(!c.primary_valid && !c.backup_valid); /* no healthy uplink */
	for (unsigned h=0; h<11; h++) {
		rc.t[h].direct_permille=1000; rc.t[h].rssi=40+h;
	}
	relay_coord_learn_tick(&rc, 1000);
	relay_coord_plan_watch(&rc, &c);
	unsigned load[RC_MAX]={0};
	for (unsigned child=0; child<11; child++) {
		for (unsigned k=0; k<2; k++) {
			if (!((k ? c.backup_valid : c.primary_valid) & (1U<<child))) continue;
			unsigned h=(c.helpers[child]>>(4*k))&15;
			assert(h+1<child || child+1<h);
			assert(++load[h]<=3);
		}
	}
	/* Slot adjacency uses member positions, not numerical IDs. */
	rc.learn_members = (1U<<0)|(1U<<4)|(1U<<9);
	relay_coord_learn_tick(&rc, 2000);
	c.members=rc.learn_members; relay_coord_plan_watch(&rc,&c);
	assert((c.primary_valid&1) && (c.helpers[0]&15)==9);
	assert(!(c.primary_valid&(1U<<4)) && !(c.backup_valid&(1U<<4)));
}

static void surveys_and_grants(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	struct relay_control c={.members=7,.repair_count=5};
	for (unsigned child=0; child<3; child++) {
		c.frame=child*10;
		relay_coord_build_grants(&rc,&c,0);
		assert(c.survey==(1U<<child));
	}
	c.frame=31; c.missing[0]=1;
	relay_coord_build_grants(&rc,&c,0);
	assert(c.survey==1); /* no helper */
	c.frame=32;c.missing[0]=0;c.missing[1]=7;
	relay_coord_build_grants(&rc,&c,0);
	assert(c.survey==3); /* both paths silent, at most two */
	memset(c.missing,0,sizeof(c.missing));
	c.frame=40;c.members=0x7ff;
	/* All five repair slots occupied: no report timestamp advances. */
	rc.watch[40%RELAY_V3_CACHE_FRAMES]=(struct rc_watch){.frame=40,.primary_valid=0x1f};
	for(unsigned child=0;child<5;child++) rc.watch[40%RELAY_V3_CACHE_FRAMES].helpers[child]=child+6;
	c.missing[0]=0x1f;
	relay_coord_build_grants(&rc,&c,1000);
	for(unsigned h=0;h<RC_MAX;h++) assert(!rc.report_ms[h]);
	for(unsigned slot=0;slot<5;slot++) assert(c.grants[slot].children);
	c.frame=45;memset(c.missing,0,sizeof(c.missing));
	for(unsigned h=0;h<3;h++) {
		relay_coord_build_grants(&rc,&c,1000);
		unsigned reports=0;
		for(unsigned slot=0;slot<5;slot++) {
			if(c.grants[slot].owner<RC_MAX && !c.grants[slot].children) {
				assert(c.grants[slot].owner==h); reports++;
			}
		}
		assert(reports==1 && rc.report_ms[h]==1000);
	}
	puts("learned helpers: directed evidence, negative reports, aging, hysteresis, coverage, surveys and report grants passed");
}
/* Negative capture evidence must remove even the best remaining candidate.
 * Unknown peers can survey provisionally; known silent peers cannot consume
 * repair grants until new positive evidence or expiry permits a retry. */
static void silent_helpers(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	rc.learn_members = 31;
	for (unsigned h = 2; h < 5; h++) rc.t[h].direct_permille = 1000;
	evidence(&rc, 2, 0, 30, 0, 0, 0);
	evidence(&rc, 3, 0, 30, 0, 0, 0);
	evidence(&rc, 4, 0, 30, 0, 0, 0);
	relay_coord_learn_tick(&rc, 0);
	assert(!rc.choice[0][0].valid && !rc.choice[0][1].valid);
	evidence(&rc, 4, 0, 30, 29, 50, 100);
	relay_coord_learn_tick(&rc, 100);
	assert(rc.choice[0][0].valid && rc.choice[0][0].helper == 4);
	assert(!rc.choice[0][1].valid);
	/* Incumbent loses all capture evidence while the alternate recovers. */
	rc.peer[4][0].heard = 0;
	evidence(&rc, 3, 0, 30, 29, 55, 200);
	relay_coord_learn_tick(&rc, 200);
	assert(rc.choice[0][0].valid && rc.choice[0][0].helper == 3);
	assert(!rc.choice[0][1].valid);
}
static void weak_backup_capacity(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	rc.learn_members = (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3) | (1U << 6) | (1U << 9);
	for (unsigned h = 0; h < RC_MAX; h++) rc.t[h].direct_permille = 0;
	rc.t[0].direct_permille = rc.t[3].direct_permille = rc.t[6].direct_permille = 1000;
	const uint8_t pairs[][3] = {{0,3,6},{1,3,6},{2,0,6},{3,0,255},{6,0,255},{9,3,255}};
	for (unsigned i = 0; i < sizeof(pairs)/sizeof(*pairs); i++)
		for (unsigned k = 0; k < 2; k++)
			rc.choice[pairs[i][0]][k] = (struct rc_choice){.valid = pairs[i][k+1] != 255,
				.helper = pairs[i][k+1] == 255 ? 0 : pairs[i][k+1]};
	/* All nine helper places are reserved, but the blocked child lacks a backup. */
	relay_coord_learn_tick(&rc, 0);
	assert(rc.choice[9][0].valid && rc.choice[9][1].valid);
	assert(rc.choice[9][0].helper != rc.choice[9][1].helper);
	unsigned load[RC_MAX] = {0};
	for (unsigned c = 0; c < RC_MAX; c++)
		for (unsigned k = 0; k < 2; k++) {
			struct rc_choice *q = &rc.choice[c][k];
			if (!q->valid) continue;
			assert(rc.t[q->helper].direct_permille >= 950);
			assert(++load[q->helper] <= RELAY_MAX_CHILDREN);
			assert(relay_can_overhear(rc.learn_members, q->helper, c));
		}
}

static void primary_before_backup(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	rc.learn_members = 0x7ff;
	/* Six places, nine weak children: do not spend any on a backup. */
	rc.t[8].direct_permille = rc.t[10].direct_permille = 1000;
	for (unsigned c = 0; c < 3; c++) {
		rc.choice[c][0] = (struct rc_choice){.valid=true,.helper=8};
		rc.choice[c][1] = (struct rc_choice){.valid=true,.helper=10};
	}
	relay_coord_learn_tick(&rc, 0);
	unsigned primary = 0, backup = 0;
	for (unsigned c = 0; c < 8; c++) {
		primary += rc.choice[c][0].valid;
		backup += rc.choice[c][1].valid;
	}
	assert(primary == 6 && backup == 0);
}

static void augment_stranded_capacity(void)
{
	struct relay_coord rc;
	relay_coord_init(&rc);
	rc.learn_members = 0x7ff;
	rc.t[8].direct_permille = rc.t[10].direct_permille = 1000;
	/* Child 3 can use only 8. Move an earlier primary to 10 to fit it. */
	for (unsigned c = 0; c < 3; c++)
		rc.choice[c][0] = (struct rc_choice){.valid=true,.helper=8};
	rc.peer[10][3] = (struct rc_peer){.expected=30};
	relay_coord_learn_tick(&rc, 0);
	for (unsigned c = 0; c < 4; c++) assert(rc.choice[c][0].valid);
	assert(rc.choice[3][0].helper == 8);
}

static void allocation_invariants(void)
{
	uint32_t seed = 731;
	for (unsigned trial = 0; trial < 10000; trial++) {
		struct relay_coord rc;
		relay_coord_init(&rc);
		rc.learn_members = 0x7ff;
		for (unsigned h = 0; h < 11; h++) {
			seed = seed * 1664525U + 1013904223U;
			rc.t[h].direct_permille = seed & 0x10000 ? 1000 : 0;
			for (unsigned c = 0; c < 11; c++) {
				seed = seed * 1664525U + 1013904223U;
				rc.peer[h][c] = (struct rc_peer){.expected=30,.heard=(seed >> 16) % 3 ? 29 : 0};
			}
		}
		for (unsigned pass = 0; pass < 3; pass++) {
			relay_coord_learn_tick(&rc, pass * 500);
			unsigned load[RC_MAX] = {0};
			for (unsigned c = 0; c < RC_MAX; c++) {
				for (unsigned k = 0; k < 2; k++) {
					struct rc_choice *q = &rc.choice[c][k];
					if (!q->valid) continue;
					assert(relay_can_overhear(rc.learn_members, q->helper, c));
					assert(rc.t[q->helper].direct_permille >= 950);
					assert(rc.peer[q->helper][c].heard);
					assert(++load[q->helper] <= RELAY_MAX_CHILDREN);
				}
				assert(!rc.choice[c][1].valid || (rc.choice[c][0].valid &&
					rc.choice[c][0].helper != rc.choice[c][1].helper));
			}
		}
	}
}

int main(void) {
	allocation_invariants(); primary_before_backup(); augment_stranded_capacity(); weak_backup_capacity(); silent_helpers(); learning(); rssi_hold(); coverage(); surveys_and_grants(); }
