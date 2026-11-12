#include "grants_support.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "relay_coord.h"

static struct relay_coord rc;
static struct relay_control c;
static struct rc_grant_plan p;
static void setup(void)
{
 relay_coord_init(&rc);
 c=(struct relay_control){.members=0x7ff,.expected=1U<<9,.frame=100,
  .session=42,.epoch=3,.repair_count=5,.profile=1};
 rc.learn_members=c.members;
 for(unsigned h=0;h<RC_MAX;h++)rc.maintenance_ms[h]=rc.report_ms[h]=5000;
 for(unsigned age=0;age<3;age++){
  unsigned frame=c.frame-age,i=frame%3;
  rc.watch[i]=(struct rc_watch){.frame=frame,.session=42,.epoch=3,
   .members=c.members,.survey=1U<<9,.primary_valid=1U<<9,.backup_valid=1U<<9};
  rc.watch[i].helpers[9]=0x75;
  rc.receipt[i]=(struct rc_receipt){.frame=frame};
  c.missing[age]=1U<<9;
 }
 rc.t[2].direct_permille=950;
}
static void plan(void){relay_coord_plan_grants(&rc,&c,&p,5000);}
static unsigned rescue_slots(void){
 unsigned count=0;
 for(unsigned i=0;i<c.repair_count;i++) count+=c.grants[i].owner==2 && c.grants[i].children==(1U<<9);
 return count;
}
int main(void)
{
	setup();
	rc.t[3].direct_permille = 1000;
	rc.peer[2][9] = (struct rc_peer){.expected = 100, .heard = 96, .rssi_x8 = 20 * 8};
	rc.peer[3][9] = (struct rc_peer){.expected = 100, .heard = 99, .rssi_x8 = 25 * 8};
	/* Five slots retain cedeea0's within-50-permille RSSI comparison. */
	plan();
	assert(p.rescue_helper == 2);
	setup();
	plan();
	assert(p.rescue_child == (1U << 9) && p.rescue_helper == 2 && p.rescue_age == 2);
	relay_coord_finish_grants(&rc, &c, &p);
	assert(rescue_slots() == 1);
	relay_coord_commit_grants(&rc, &c, &p);
	struct rc_watch *w = &rc.watch[(c.frame + 1) % 3];
	assert(w->survey == c.survey && w->session == 42 && w->epoch == 3 && w->members == c.members);
	/* Exact-frame, session, epoch and membership fencing, never RSSI alone. */
	for (unsigned fault = 0; fault < 7; fault++) {
		setup();
		for (unsigned i = 0; i < 3; i++) {
			if (fault == 0) {
				rc.watch[i].frame += 3;
			}
			if (fault == 1) {
				rc.watch[i].session++;
			}
			if (fault == 2) {
				rc.watch[i].epoch++;
			}
			if (fault == 3) {
				rc.watch[i].members ^= 1;
			}
			if (fault == 4) {
				rc.watch[i].survey = 0;
			}
			if (fault == 5) {
				rc.receipt[i].received = 1U << 9;
			}
			if (fault == 6) {
				rc.watch[i].helpers[9] = 0x72;
			}
		}
		plan();
		assert(!p.rescue_child);
	}
 setup();rc.t[2].direct_permille=949;plan();assert(!p.rescue_child);
 setup();rc.peer[2][9].expected=30;plan();assert(!p.rescue_child);
 setup();rc.t[2].direct_permille=0;rc.t[8].direct_permille=1000;plan();assert(!p.rescue_child);
 /* A late repair suppresses the speculative grant at freeze. */
 setup();plan();rc.receipt[(c.frame-2)%3].received=1U<<9;
 relay_coord_finish_grants(&rc,&c,&p);assert(!rescue_slots());
 /* Reports, urgent maintenance and distinct repair reservations win. */
 setup();rc.report_ms[4]=0;plan();relay_coord_finish_grants(&rc,&c,&p);
 assert(c.grants[2].owner==4 && c.grants[2].children==0 && rescue_slots()==1);
 setup();rc.maintenance_ms[0]=0;rc.report_ms[4]=0;plan();relay_coord_finish_grants(&rc,&c,&p);
 assert(c.grants[0].owner==RELAY_OWNER_MAINTENANCE && c.grants[1].owner==RELAY_OWNER_MAINTENANCE);
 assert(c.grants[4].owner==4 && !rescue_slots());
 /* An already missing fresh frame may use spare capacity too. The original
  * pair still owns its distinct reservations before any surveyed alternate. */
 setup();rc.receipt[(c.frame-1)%3].received=rc.receipt[(c.frame-2)%3].received=1U<<9;
 plan();assert(p.rescue_child==(1U<<9) && p.rescue_age==0 && p.rescue_helper==2);
 relay_coord_finish_grants(&rc,&c,&p);assert(rescue_slots()==1);
 assert(c.grants[0].owner==5 && c.grants[1].owner==7);
 /* A direct receipt between planning and freeze cancels the fresh rescue. */
 setup();rc.receipt[(c.frame-1)%3].received=rc.receipt[(c.frame-2)%3].received=1U<<9;
 plan();rc.receipt[c.frame%3].received=1U<<9;
 relay_coord_finish_grants(&rc,&c,&p);assert(!rescue_slots());
 /* A working pair must not suppress survey copies for a persistently weak
  * direct uplink. Keep the 950 threshold and the two-survey bound. */
 for(unsigned proof=949;proof<=950;proof++){
  setup();c.primary_valid=c.backup_valid=1U<<9;
  rc.t[9].direct_permille=proof;c.missing[1]=c.missing[2]=0;
  plan();relay_coord_finish_grants(&rc,&c,&p);
  assert(!!(c.survey&(1U<<9))==(proof<950));
  assert(relay_mask_count(c.survey)<=2);
 }
 setup();c.primary_valid=c.backup_valid=1U<<9;c.missing[0]=c.missing[1]=c.missing[2]=0;
 plan();relay_coord_finish_grants(&rc,&c,&p);assert(!(c.survey&(1U<<9)));
 /* A speculative helper can have an empty cache. Prefer another eligible
  * survey copy at age two, without dropping the sole-candidate fallback. */
 setup();
 rc.t[3].direct_permille=1000;

 rc.watch[(c.frame - 2) % 3].rescue_tried[9] = 1U << 2;
 plan();assert(p.rescue_helper==3);
 rc.t[3].direct_permille=949;
 plan();assert(p.rescue_helper==2);
 /* Both spare slots may hold distinct alternates, after the due report. */
 setup();rc.t[3].direct_permille=1000;rc.report_ms[4]=0;
 plan();relay_coord_finish_grants(&rc,&c,&p);
 assert(c.grants[2].owner==4 && !c.grants[2].children);
 assert(c.grants[3].owner==2 && c.grants[3].children==(1U<<9));
 assert(c.grants[4].owner==3 && c.grants[4].children==(1U<<9));
 relay_coord_commit_grants(&rc,&c,&p);
 /* Exercise an age-one reservation so its history survives commit. */
 setup();rc.t[3].direct_permille=1000;
 rc.receipt[(c.frame-2)%3].received=1U<<9;
 plan();relay_coord_finish_grants(&rc,&c,&p);relay_coord_commit_grants(&rc,&c,&p);
 assert(rc.watch[(c.frame - 1) % 3].rescue_tried[9] == ((1U << 2) | (1U << 3) | (1U << 5) | (1U << 7)));
 c.frame++;rc.t[4].direct_permille=1000;
 plan();assert(p.rescue_age==2 && p.rescue_helper==4);
 /* Only a committed reservation consumes the first alternate. */
 setup();rc.receipt[(c.frame-2)%3].received=1U<<9;
 plan();assert(p.rescue_age==1);
 relay_coord_finish_grants(&rc,&c,&p);relay_coord_commit_grants(&rc,&c,&p);

 assert(rc.watch[(c.frame - 1) % 3].rescue_tried[9] == ((1U << 2) | (1U << 5) | (1U << 7)));
 setup();rc.maintenance_ms[0]=0;rc.report_ms[4]=0;
 rc.receipt[(c.frame-2)%3].received=1U<<9;
 plan();relay_coord_finish_grants(&rc,&c,&p);relay_coord_commit_grants(&rc,&c,&p);
 assert(!(rc.watch[(c.frame - 1) % 3].rescue_tried[9] & (1U << 2)));
 /* The B plan first used in frame 99 can also cover frame 101 after two
  * missed B packets. Preserve that opportunity, but never extend inherited
  * listeners or cross an epoch/session/membership boundary. */
 for(unsigned fault=0;fault<8;fault++) {
  setup();rc.t[2].direct_permille=0;rc.t[4].direct_permille=1000;
  c.primary_valid=c.backup_valid=1U<<9;c.helpers[9]=0x75;
  for(unsigned i=0;i<3;i++){rc.watch[i].survey=0;c.missing[i]=0;}
  struct rc_watch *old=&rc.watch[(c.frame-1)%3];old->helpers[9]=0x74;
  if(fault==1)old->epoch++;
  if(fault==2)old->session++;
  if(fault==3)old->members^=1;
  if(fault==4)old->frame-=3;
  if(fault==5){old->helpers[9]=0x75;rc.watch[(c.frame-2)%3].helpers[9]=0x74;}
  if(fault==6){old->helpers[9]=0x75;rc.watch[c.frame%3].listeners[9]=1U<<4;}
  if(fault==7){old->helpers[9]=0x75;old->survey=1U<<9;}
  plan();relay_coord_finish_grants(&rc,&c,&p);relay_coord_commit_grants(&rc,&c,&p);
  bool eligible=fault==0 || fault==7;
  assert(!!(rc.watch[(c.frame+1)%3].listeners[9]&(1U<<4))==eligible);
  rc.receipt[c.frame%3].received=1U<<9;
  c.frame+=2;plan();
  assert(!!p.rescue_child==eligible);
  if(eligible)assert(p.rescue_helper==4 && p.rescue_age==1);
 }
 /* Intermittent weak paths need listeners before their next isolated miss.
  * Keep the existing threshold, periodic exploration and two-survey cap. */
 setup();
 rc.t[9].direct_permille = 1000;
 c.frame = 101;
 memset(c.missing, 0, sizeof(c.missing));
 rc.survey_next = 7;
 rc.t[0].direct_permille=949;rc.t[1].direct_permille=950;
 plan();assert(p.survey==1U && p.survey_next==7);
 relay_coord_finish_grants(&rc,&c,&p);assert(c.survey==1U);
 setup();
 rc.t[9].direct_permille = 1000;
 c.frame = 101;
 memset(c.missing, 0, sizeof(c.missing));
 rc.t[0].direct_permille=950;plan();assert(!p.survey);
 rc.t[0].direct_permille=0;plan();assert(!p.survey);
 rc.t[0].direct_permille=949;rc.t[1].direct_permille=900;
 plan();assert(p.survey==2U);
 setup();
 rc.t[9].direct_permille = 1000;
 rc.survey_next = 7;
 rc.t[0].direct_permille = 949;
 plan();assert(p.survey==(1U<<7) && p.survey_next==8);
 setup();
 rc.t[9].direct_permille = 1000;
 c.frame = 101;
 rc.t[0].direct_permille = 949;
 plan();relay_coord_finish_grants(&rc,&c,&p);
 assert(c.survey==(1U|(1U<<9)) && relay_mask_count(c.survey)==2);
 /* Aged paired repairs lead with backup, leaving primary a later slot.
  * Fresh and unpaired reservations keep their original helper. */
 setup();
 plan();
 assert(p.helper[0][9] == 7 && p.helper[0][25] == 7 && p.helper[1][9] == 5);
 assert(p.helper[1][25] == 5 && p.helper[2][9] == 5 && p.helper[2][25] == 7);
 relay_coord_finish_grants(&rc, &c, &p);
 assert(c.grants[0].owner == 7 && c.grants[1].owner == 5);
 setup();
 rc.watch[(c.frame - 2) % 3].backup_valid = 0;
 plan();
 assert(p.helper[0][9] == 5);
 /* Persistent drops: an oldest-frame grant can transmit any cached frame.
  * Surviving records must rotate, and children must not share tried masks. */
 setup();
 rc.t[3].direct_permille = 1000;
 rc.t[4].direct_permille = 1000;
 plan();
 relay_coord_finish_grants(&rc, &c, &p);
 relay_coord_commit_grants(&rc, &c, &p);
 for (unsigned age = 0; age < 2; age++) {
	 assert((rc.watch[(c.frame - age) % 3].rescue_tried[9] & 12U) == 12U);
	 assert(!rc.watch[(c.frame - age) % 3].rescue_tried[3]);
 }
 c.frame++;
 plan();
 assert(p.rescue_helper == 4);
 /* A grant for another child, including a helper shared with child 9,
  * only consumes that child's opportunity; no cross-session attribution. */
 setup();
 plan();
 relay_coord_finish_grants(&rc, &c, &p);
 c.grants[0] = (struct relay_grant){.owner = 4, .children = 1U << 3};
 c.missing[1] |= 1U << 3;
 rc.watch[(c.frame - 1) % 3].session++;
 relay_coord_commit_grants(&rc, &c, &p);
 assert(!rc.watch[(c.frame - 1) % 3].rescue_tried[3]);
 assert(!(rc.watch[c.frame % 3].rescue_tried[9] & (1U << 4)));
 /* An untried planned alternate takes the failed earlier lane only. */
 setup();
 rc.watch[(c.frame - 2) % 3].rescue_tried[9] = 1U << 7;
 plan();
 assert(p.helper[0][9] == 2 && p.helper[1][25] == 5);
 relay_coord_finish_grants(&rc, &c, &p);
 assert(c.grants[0].owner == 2 && (c.grants[0].children & (1U << 9)));
 setup();
 rc.watch[(c.frame - 2) % 3].rescue_tried[9] = (1U << 7) | (1U << 2);
 plan();
 assert(p.helper[0][9] == 7);
 setup();
 rc.watch[(c.frame - 2) % 3].rescue_tried[9] = 1U << 7;
 rc.t[2].direct_permille = 949;
 plan();
 assert(p.helper[0][9] == 7);
 setup();
 rc.watch[(c.frame - 2) % 3].rescue_tried[3] = 1U << 7;
 plan();
 assert(p.helper[0][9] == 7);
 /* The same retained pair gets a later first slot on alternate frames. */
 setup();
 plan();
 assert(p.helper[0][9] == 7);
 c.frame++;
 plan();
 assert(p.helper[0][9] == 5 && p.helper[1][25] == 7);
 /* Active zero-direct sources receive surveys; absent zeros do not. */
 setup();
 c.frame = 101;
 c.expected = (1U << 3) | (1U << 9);
 plan();
 assert(p.survey == (1U << 3) && p.survey_next == 4);
 rc.survey_next = p.survey_next;
 plan();
 assert(p.survey == (1U << 9) && p.survey_next == 10);
 c.expected = 0;
 plan();
 assert(!p.survey);
 puts("Survey rescue: exact history, qualification, late receipt and priority passed");
}
