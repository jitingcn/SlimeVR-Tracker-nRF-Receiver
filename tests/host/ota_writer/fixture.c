#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...)
#define LOG_DBG(...)
#define LOG_WRN(...)
#define LOG_ERR(...)
#define ARG_UNUSED(x) (void)(x)
#define __ASSERT_NO_MSG(x) assert(x)
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define CONFIG_SOC_NRF52840 1
#if TEST_MCUBOOT
#define CONFIG_BOOTLOADER_MCUBOOT 1
#endif
#if TEST_UF2
#define CONFIG_BUILD_OUTPUT_UF2 1
#endif
#define CONFIG_BOARD "probe"
#define DT_NODE_EXISTS(x) TEST_MCUBOOT
#define DT_NODELABEL(x) 0
#define DT_CHOSEN(x) 0
#define PARTITION_NODE_OFFSET(x) 0x1000
#define PARTITION_ID(x) 0
#define RCV_OTA_WRITER_PRIORITY 5
#define RECEIVER_OTA_ID 0xFE
#define K_FOREVER (-1)
#define K_NO_WAIT 0
#define K_MSEC(x) (x)
#define K_THREAD_STACK_DEFINE(n,s) char n[s]
#define K_SEM_DEFINE(n,c,m) struct k_sem n = {PTHREAD_MUTEX_INITIALIZER,c}
#define K_MSGQ_DEFINE(n,s,c,a) struct k_msgq n = {PTHREAD_MUTEX_INITIALIZER,PTHREAD_COND_INITIALIZER,{0},0,0}
typedef atomic_int atomic_t;
#define atomic_get(a) atomic_load(a)
#define atomic_set(a,v) atomic_store(a,v)
#define atomic_clear(a) atomic_store(a,0)
static bool radio_owned;
static int esb_radio_operation_begin(void) {
    if (radio_owned) return -EBUSY;
    radio_owned = true;
    return 0;
}
static void esb_radio_operation_end(void) { assert(radio_owned); radio_owned = false; }
static void esb_auto_loss_invalidate(void) {}
struct device { int unused; } device;
#define DEVICE_DT_GET(x) (&device)
static bool device_is_ready(const struct device *d) { return true; }
static int64_t k_uptime_get(void) { return 123; }
static void k_msleep(int ms) {}
static uint16_t sys_get_be16(const uint8_t *p) { return (p[0]<<8)|p[1]; }
static uint32_t sys_get_le32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void sys_put_be16(uint16_t n,uint8_t *p) { p[0]=n>>8; p[1]=n; }
static void sys_put_le32(uint32_t n,uint8_t *p) { for(int i=0;i<4;i++) p[i]=n>>(8*i); }
static uint32_t crc32_ieee_update(uint32_t c,const uint8_t *p,size_t n) {
    c=~c; while(n--) { c^=*p++; for(int i=0;i<8;i++) c=(c>>1)^((0-(c&1))&0xEDB88320); } return ~c;
}
static uint8_t last_status[16];
static void (*status_observer)(const uint8_t *);
static void (*status_entry_hook)(void);
static void hid_write_packet_n(const uint8_t *p,int n) {
    memcpy(last_status,p,16);
    if(status_observer) status_observer(p);
}
struct k_sem { pthread_mutex_t lock; int count; };
static unsigned slot_resets;
static void k_sem_reset(struct k_sem *s) { pthread_mutex_lock(&s->lock); s->count=0; slot_resets++; pthread_mutex_unlock(&s->lock); }
static void k_sem_give(struct k_sem *s) { pthread_mutex_lock(&s->lock); s->count=1; pthread_mutex_unlock(&s->lock); }
static int k_sem_take(struct k_sem *s,int timeout) {
    pthread_mutex_lock(&s->lock); int ret=s->count?0:-EBUSY; if(!ret) s->count--; pthread_mutex_unlock(&s->lock); return ret;
}
struct k_msgq { pthread_mutex_t lock; pthread_cond_t wake; uint8_t data[4]; unsigned count,first; };
static unsigned purges;
static int fail_command=-1;
static unsigned command_puts[256];
static void (*before_put)(uint8_t), (*after_put)(uint8_t);
static void k_msgq_purge(struct k_msgq *q) { pthread_mutex_lock(&q->lock); q->count=q->first=0; purges++; pthread_mutex_unlock(&q->lock); }
static int k_msgq_put(struct k_msgq *q,const uint8_t *p,int timeout) {
    if(before_put) before_put(*p);
    command_puts[*p]++;
    if(*p==fail_command) return -ENOMSG;
    pthread_mutex_lock(&q->lock); if(q->count==4) { pthread_mutex_unlock(&q->lock); return -ENOMSG; }
    q->data[(q->first+q->count++)%4]=*p; pthread_cond_signal(&q->wake); pthread_mutex_unlock(&q->lock);
    if(after_put) after_put(*p);
    return 0;
}
static int k_msgq_get(struct k_msgq *q,uint8_t *p,int timeout) {
    pthread_mutex_lock(&q->lock); while(!q->count) pthread_cond_wait(&q->wake,&q->lock);
    *p=q->data[q->first]; q->first=(q->first+1)%4; q->count--; pthread_mutex_unlock(&q->lock); return 0;
}
struct k_thread { pthread_t id; void (*fn)(void *,void *,void *); bool joined; };
static unsigned creates, joins, join_failures;
static pthread_mutex_t gate_lock=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_cond=PTHREAD_COND_INITIALIZER;
static bool hold_write, hold_read, leaf_entered, release_leaf, thread_exited;
static void *thread_entry(void *arg) {
    struct k_thread *t=arg; t->fn(NULL,NULL,NULL);
    pthread_mutex_lock(&gate_lock); thread_exited=true; pthread_cond_broadcast(&gate_cond); pthread_mutex_unlock(&gate_lock); return NULL;
}
static void k_thread_create(struct k_thread *t,void *stack,int size,void (*fn)(void *,void *,void *),void *a,void *b,void *c,int prio,int flags,int delay) {
    assert(!creates || t->joined); creates++; t->joined=false; t->fn=fn; thread_exited=false; assert(!pthread_create(&t->id,NULL,thread_entry,t));
}
static void k_thread_name_set(struct k_thread *t,const char *name) {}
static int k_thread_join(struct k_thread *t,int timeout) {
    assert(timeout>0 && timeout<=5000); joins++;
    if(join_failures) { join_failures--; return -EAGAIN; }
    assert(!pthread_join(t->id,NULL)); t->joined=true; return 0;
}
static void pause_leaf(bool hold) {
    if(!hold) return;
    pthread_mutex_lock(&gate_lock); leaf_entered=true; pthread_cond_broadcast(&gate_cond);
    while(!release_leaf) pthread_cond_wait(&gate_cond,&gate_lock);
    pthread_mutex_unlock(&gate_lock);
}
static void wait_leaf(void) { pthread_mutex_lock(&gate_lock); while(!leaf_entered) pthread_cond_wait(&gate_cond,&gate_lock); pthread_mutex_unlock(&gate_lock); }
static void release_and_wait(void) {
    pthread_mutex_lock(&gate_lock); release_leaf=true; pthread_cond_broadcast(&gate_cond);
    while(!thread_exited) pthread_cond_wait(&gate_cond,&gate_lock);
    pthread_mutex_unlock(&gate_lock);
}
static uint8_t staging[8192];
static unsigned erases,writes,reads,reboots;
static unsigned copies, fail_read_call;
static uint32_t read_base;
static int upgrade_error;
static int flash_erase(const struct device *d,uint32_t addr,size_t size) { erases++; memset(staging,0xff,sizeof(staging)); return 0; }
static int flash_flatten(const struct device *d,uint32_t addr,size_t size) { return flash_erase(d,addr,size); }
static int flash_write(const struct device *d,uint32_t addr,const void *p,size_t size) {
    uint8_t admitted[4096]; memcpy(admitted,p,size); pause_leaf(hold_write);
    assert(!memcmp(admitted,p,size)); memcpy(staging,p,size); writes++; return 0;
}
static int flash_read(const struct device *d,uint32_t addr,void *p,size_t size) {
    pause_leaf(hold_read);
    reads++;
    if (reads==fail_read_call) return -EIO;
    size_t offset=read_base?addr-read_base:0;
    assert(offset+size<=sizeof(staging));
    memcpy(p,staging+offset,size);
    return 0;
}
struct flash_area { uint32_t fa_off,fa_size; } secondary_area={0x80000,8192};
static int flash_area_open(int id,const struct flash_area **a) { *a=&secondary_area; return 0; }
static void flash_area_close(const struct flash_area *a) {}
static size_t boot_get_image_start_offset(int id) { return 0; }
static ssize_t boot_get_area_trailer_status_offset(int id) { return 8192; }
static bool boot_is_img_confirmed(void) { return true; }
#define BOOT_UPGRADE_TEST 0
#define SYS_REBOOT_COLD 0
static int boot_request_upgrade(int mode) { return upgrade_error; }
static void sys_reboot(int type) { reboots++; }

/* PRODUCTION_SOURCE */

static void rcv_ota_activate_and_reset(void) {
#if TEST_UF2
    copies++;
#else
    assert(!"hardware RAM copier is outside this probe");
#endif
}
static void command(uint8_t type) { uint8_t p[2]={type,RECEIVER_OTA_ID}; receiver_ota_process_hid(p,sizeof(p)); }
static void begin(uint32_t size) {
    uint8_t p[64]={HID_OTA_BEGIN,RECEIVER_OTA_ID}; sys_put_le32(size,p+2);
    sys_put_be16((size+59)/60,p+10); p[12]=OTA_PROTOCOL_VERSION; memcpy(p+13,CONFIG_BOARD,sizeof(CONFIG_BOARD));
    receiver_ota_process_hid(p,sizeof(p));
}
static void start_busy_write(void) {
    begin(4096); assert(rcv_ota.state==RCV_OTA_READY); hold_write=true;
    memset(rcv_ota.page_buf,0x5a,4096); rcv_ota.page_buf_offset=4096;
    assert(!rcv_ota_flush_page_buf()); wait_leaf();
}
static void pending_admission(void) {
    uint8_t p[64]={HID_OTA_DATA,RECEIVER_OTA_ID};
    receiver_ota_process_hid(p,sizeof(p)); command(HID_OTA_VERIFY); command(HID_OTA_ACTIVATE);
}
static void timeout_case(bool aborting) {
    start_busy_write();
    if(!aborting) {
        /* A real sequence error makes a replacement BEGIN admissible. */
        uint8_t bad[8]={HID_OTA_DATA,RECEIVER_OTA_ID,0,1}; receiver_ota_process_hid(bad,sizeof(bad));
        assert(rcv_ota.state==RCV_OTA_ERROR);
    }
    /* Full queue: failed shutdown enqueue must not prevent retirement. */
    uint8_t cmd=WRITER_CMD_VERIFY; for(int i=0;i<4;i++) assert(!k_msgq_put(&page_write_msgq,&cmd,0));
    typeof(rcv_ota) saved=rcv_ota;
    struct page_write_req saved_reqs[2]; memcpy(saved_reqs,page_write_reqs,sizeof(saved_reqs));
    unsigned old_erases=erases,old_resets=slot_resets,old_purges=purges,old_creates=creates;
    join_failures=2;
    if(aborting) command(HID_OTA_ABORT); else begin(2048);
    assert(last_status[2]==OTA_STATUS_TIMEOUT);
    assert(!memcmp(&saved,&rcv_ota,sizeof(saved)));
    pending_admission();
    assert(!memcmp(&saved,&rcv_ota,sizeof(saved)));
    /* A second BEGIN must retry join even after the cooperative flag is off. */
    begin(2048);
    assert(joins==2 && last_status[2]==OTA_STATUS_TIMEOUT);
    assert(!memcmp(&saved,&rcv_ota,sizeof(saved)));
    assert(!memcmp(saved_reqs,page_write_reqs,sizeof(saved_reqs)));
    assert(erases==old_erases && slot_resets==old_resets && purges==old_purges && creates==old_creates);
    release_and_wait(); assert(writes==1);
    if(aborting) { command(HID_OTA_ABORT); assert(rcv_ota.state==RCV_OTA_IDLE); }
    else { begin(2048); assert(rcv_ota.state==RCV_OTA_READY && rcv_ota.image_size==2048); command(HID_OTA_ABORT); }
    assert(rcv_ota_writer_thread.joined);
}
static void verify_timeout(void) {
    begin(256); memset(staging,0x33,256); hold_read=true;
    rcv_ota.image_crc32=crc32_ieee_update(0,staging,256); rcv_ota.bytes_written=256;
    command(HID_OTA_VERIFY); wait_leaf(); typeof(rcv_ota) saved=rcv_ota;
    join_failures=2; command(HID_OTA_ABORT);
    assert(last_status[2]==OTA_STATUS_TIMEOUT && !memcmp(&saved,&rcv_ota,sizeof(saved)));
    begin(512); assert(!memcmp(&saved,&rcv_ota,sizeof(saved)));
    release_and_wait(); assert(reads==1 && rcv_ota.error_code==OTA_STATUS_VERIFY_OK);
    /* The finishing verifier must not replace the outstanding stop error. */
    assert(last_status[2]==OTA_STATUS_TIMEOUT);
    begin(512); assert(rcv_ota.state==RCV_OTA_READY && rcv_ota.image_size==512); command(HID_OTA_ABORT);
}
static void activation(bool error) {
    begin(256); rcv_ota.state=RCV_OTA_VERIFYING;
    rcv_ota.error_code=OTA_STATUS_VERIFY_OK; upgrade_error=error?-EIO:0;
    command(HID_OTA_ACTIVATE);
    pthread_mutex_lock(&gate_lock); while(!thread_exited) pthread_cond_wait(&gate_cond,&gate_lock); pthread_mutex_unlock(&gate_lock);
    assert(rcv_ota.state==(error?RCV_OTA_ERROR:RCV_OTA_COMPLETE)); assert(reboots==!error);
    begin(512); assert(joins==1 && creates==2 && rcv_ota.state==RCV_OTA_READY); command(HID_OTA_ABORT);
}
static bool verify_status_waiting, release_verify_status, verify_status_sent;
static unsigned activation_entries;
static void wait_for_exit(void) {
    pthread_mutex_lock(&gate_lock);
    while(!thread_exited) pthread_cond_wait(&gate_cond,&gate_lock);
    pthread_mutex_unlock(&gate_lock);
}
static void pause_verified_status(void) {
    if(!pthread_equal(pthread_self(),rcv_ota_writer_thread.id) ||
       rcv_ota.state!=RCV_OTA_VERIFYING || rcv_ota.error_code!=OTA_STATUS_VERIFY_OK) return;
    pthread_mutex_lock(&gate_lock);
    verify_status_waiting=true; pthread_cond_broadcast(&gate_cond);
    while(!release_verify_status) pthread_cond_wait(&gate_cond,&gate_lock);
    pthread_mutex_unlock(&gate_lock);
}
static void observe_admission_status(const uint8_t *p) {
    if(p[2]==OTA_STATUS_RECEIVING && pthread_equal(pthread_self(),rcv_ota_writer_thread.id)) {
        pthread_mutex_lock(&gate_lock);
        verify_status_sent=true; pthread_cond_broadcast(&gate_cond);
        pthread_mutex_unlock(&gate_lock);
    }
    if(p[2]==OTA_STATUS_ACTIVATING && !activation_entries) {
        assert(pthread_equal(pthread_self(),rcv_ota_writer_thread.id));
        activation_entries++;
        unsigned old_puts=command_puts[WRITER_CMD_ACTIVATE];
        command(HID_OTA_ACTIVATE);
        assert(command_puts[WRITER_CMD_ACTIVATE]==old_puts);
    }
}
static void pending_interleave(uint8_t cmd) {
    if(cmd!=WRITER_CMD_ACTIVATE) return;
    assert(rcv_ota.state==RCV_OTA_ACTIVATE_PENDING);
    assert(command_puts[cmd]==0);
    command(HID_OTA_ACTIVATE);
    assert(last_status[2]==OTA_STATUS_RECEIVING && command_puts[cmd]==0);
    command(HID_OTA_VERIFY);
    assert(command_puts[WRITER_CMD_VERIFY]==1);
    pthread_mutex_lock(&gate_lock);
    release_verify_status=true; pthread_cond_broadcast(&gate_cond);
    while(!verify_status_sent) pthread_cond_wait(&gate_cond,&gate_lock);
    pthread_mutex_unlock(&gate_lock);
    assert(last_status[2]==OTA_STATUS_RECEIVING && activation_entries==0);
}
static void complete_before_return(uint8_t cmd) {
    if(cmd==WRITER_CMD_ACTIVATE) wait_for_exit();
}
static void admission_race(bool error) {
    begin(256); memset(staging,0x33,256);
    rcv_ota.image_crc32=crc32_ieee_update(0,staging,256); rcv_ota.bytes_written=256;
    status_entry_hook=pause_verified_status; status_observer=observe_admission_status;
    command(HID_OTA_VERIFY);
    pthread_mutex_lock(&gate_lock);
    while(!verify_status_waiting) pthread_cond_wait(&gate_cond,&gate_lock);
    pthread_mutex_unlock(&gate_lock);
    command(HID_OTA_VERIFY);
    assert(command_puts[WRITER_CMD_VERIFY]==1 && last_status[2]==OTA_STATUS_VERIFY_OK);
    before_put=pending_interleave; after_put=complete_before_return; upgrade_error=error?-EIO:0;
    command(HID_OTA_ACTIVATE);
    assert(activation_entries==1 && command_puts[WRITER_CMD_ACTIVATE]==1);
    assert(rcv_ota.state==(error?RCV_OTA_ERROR:RCV_OTA_COMPLETE));
    assert(last_status[2]==(error?OTA_STATUS_FLASH_ERROR:OTA_STATUS_COMPLETE));
    command(HID_OTA_ACTIVATE);
    assert(command_puts[WRITER_CMD_ACTIVATE]==1 && page_write_msgq.count==0);
    before_put=after_put=NULL; status_entry_hook=NULL; status_observer=NULL;
    command(HID_OTA_ABORT);
}
static void command_failure(bool activate) {
    begin(256); rcv_ota.bytes_written=256;
    if(activate) { rcv_ota.state=RCV_OTA_VERIFYING; rcv_ota.error_code=OTA_STATUS_VERIFY_OK; }
    fail_command=activate?WRITER_CMD_ACTIVATE:WRITER_CMD_VERIFY;
    command(activate?HID_OTA_ACTIVATE:HID_OTA_VERIFY);
    assert(rcv_ota.state==RCV_OTA_ERROR && rcv_ota.error_code==OTA_STATUS_ERROR);
    assert(last_status[2]==OTA_STATUS_ERROR && page_write_msgq.count==0);
    command(activate?HID_OTA_ACTIVATE:HID_OTA_VERIFY);
    assert(command_puts[fail_command]==1 && last_status[2]==OTA_STATUS_ERROR);
    fail_command=-1; command(HID_OTA_ABORT);
}
static void activation_eligibility(void) {
    begin(256); rcv_ota.error_code=OTA_STATUS_VERIFY_OK;
    command(HID_OTA_ACTIVATE);
    assert(rcv_ota.state==RCV_OTA_READY && command_puts[WRITER_CMD_ACTIVATE]==0);
    rcv_ota.state=RCV_OTA_VERIFYING; rcv_ota.error_code=0;
    command(HID_OTA_ACTIVATE);
    assert(rcv_ota.state==RCV_OTA_VERIFYING && command_puts[WRITER_CMD_ACTIVATE]==0);
    command(HID_OTA_ABORT);
}
/* CRC_SETTINGS_CASES */
int main(int argc,char **argv) {
    assert(argc==2);
    if(crc_settings_case(argv[1])) return 0;
    if(!strcmp(argv[1],"begin_timeout")) timeout_case(false);
    else if(!strcmp(argv[1],"abort_timeout")) timeout_case(true);
    else if(!strcmp(argv[1],"verify_timeout")) verify_timeout();
    else if(!strcmp(argv[1],"complete")) activation(false);
    else if(!strcmp(argv[1],"activation_error")) activation(true);
    else if(!strcmp(argv[1],"verify_enqueue_error")) command_failure(false);
    else if(!strcmp(argv[1],"activate_enqueue_error")) command_failure(true);
    else if(!strcmp(argv[1],"admission_complete")) admission_race(false);
    else if(!strcmp(argv[1],"admission_error")) admission_race(true);
    else if(!strcmp(argv[1],"activation_eligibility")) activation_eligibility();
    else if(!strcmp(argv[1],"data_payload")) data_payload_cases();
    else if(!strcmp(argv[1],"abort")) { command(HID_OTA_ABORT); assert(joins==0); begin(256); command(HID_OTA_ABORT); assert(joins==1 && rcv_ota.state==RCV_OTA_IDLE); begin(512); command(HID_OTA_ABORT); assert(joins==2 && creates==2); }
    else abort();
    printf("PASS %s mcuboot=%d\n",argv[1],TEST_MCUBOOT);
}
