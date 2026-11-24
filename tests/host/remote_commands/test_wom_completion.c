#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "rcv_hid_cmd.h"
#define MAX_TRACKERS 16
#define BIT(n) (1u << (n))
#define ARG_UNUSED(x) (void)(x)
#define K_MSEC(x) (x)
#define REMOTE_HID_CONFIRM_TIMEOUT_MS 5000
static char console_output[4096];
static size_t console_used;
static void capture_console(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = vsnprintf(console_output + console_used, sizeof(console_output) - console_used, format, args);
    va_end(args);
    assert(n >= 0 && (size_t)n < sizeof(console_output) - console_used);
    console_used += (size_t)n;
}
#define printk capture_console
typedef uint32_t atomic_t;
typedef uint32_t atomic_val_t;
static uint32_t atomic_get(atomic_t *p) { return *p; }
static void atomic_set(atomic_t *p, uint32_t v) { *p = v; }
static bool atomic_cas(atomic_t *p, uint32_t old, uint32_t v) { if (*p != old) return false; *p = v; return true; }
static uint32_t atomic_or(atomic_t *p, uint32_t v) { uint32_t old = *p; *p |= v; return old; }
static void sys_put_le16(uint16_t v, uint8_t *p) { p[0] = v; p[1] = v >> 8; }
static void sys_put_le32(uint32_t v, uint8_t *p) { for (int i = 0; i < 4; i++) p[i] = v >> (8 * i); }
static uint16_t sys_get_le16(const uint8_t *p) { return p[0] | (uint16_t)p[1] << 8; }
static int64_t now;
static int64_t k_uptime_get(void) { return now; }
static int remote_hid_timeout_work, remote_hid_job_work, rssi_wq;
static int scheduled, submitted;
static int k_work_cancel_delayable(int *w) { return 0; }
static int k_work_reschedule(int *w, int delay) { scheduled = delay; return 0; }
static int k_work_submit_to_queue(int *q, int *w) { submitted++; return 0; }
static atomic_t remote_hid_pending, remote_hid_expected, remote_hid_confirmed;
static uint16_t remote_hid_next_token, remote_hid_cmd_token;
static uint8_t remote_hid_seq, remote_hid_opcode, remote_hid_flag;
static bool remote_wom_console;
static uint16_t remote_wom_token, remote_wom_minutes;
static uint32_t remote_wom_failed;
static int64_t remote_wom_deadline;
enum { REMOTE_HID_JOB_NONE, REMOTE_HID_JOB_WOM_COMMAND };
static int remote_hid_job;
static uint8_t acks[20][16];
static unsigned ack_count;
static void capture(const uint8_t *ack) { assert(ack_count < 20); memcpy(acks[ack_count++], ack, 16); }
static void (*async_ack_fn)(const uint8_t *) = capture;
static uint16_t next_token, available_token[16], result_minutes[16], result_error[16];
static uint32_t available, queued_mask = BIT(2) | BIT(5), cancelled_mask;
static uint16_t cancelled_token;
static unsigned sends;
static uint16_t sent_minutes;
uint32_t esb_send_remote_wom_command(uint8_t target, uint16_t minutes, uint16_t *token)
{
    sends++;
    sent_minutes = minutes;
    *token = ++next_token;
    return target == 255 ? queued_mask : queued_mask & BIT(target);
}
bool esb_get_remote_wom_result(uint8_t id, uint16_t token, uint16_t *minutes, uint16_t *error)
{
    if (!(available & BIT(id)) || available_token[id] != token) return false;
    *minutes = result_minutes[id]; *error = result_error[id]; return true;
}
void esb_cancel_remote_wom_command(uint32_t mask, uint16_t token)
{ cancelled_mask = mask; cancelled_token = token; }
static void remote_hid_finish(uint8_t status) { assert(false); }
/* PRODUCTION BODIES */
static void reply(uint8_t id, uint16_t minutes, uint16_t error)
{
    available |= BIT(id); available_token[id] = remote_wom_token;
    result_minutes[id] = minutes; result_error[id] = error;
}
int main(void)
{
    uint8_t request[7] = {254, 7, 0x28, 0, 2, 0, 0};
    assert(hid_wom(request, 2) == RCV_HID_ST_EINVAL);
    request[5] = 0xd0; request[6] = 0x8b; /* 35792 */
    assert(hid_wom(request, 3) == RCV_HID_ST_EINVAL);
    request[5] = request[6] = 0;
    assert(hid_wom(request, 3) == RCV_HID_ST_STARTED);
    assert(remote_wom_token != 0 && scheduled == 50);
    assert(rcv_cmd_remote_wom_command(2, 10) == RCV_HID_ST_EBUSY);
    on_remote_confirm(2, ESB_PONG_FLAG_WOM_CONTROL);
    remote_wom_poll(); assert(ack_count == 0 && remote_hid_pending);
    reply(2, 17, 0); available_token[2]--; /* stale transaction */
    remote_wom_poll(); assert(ack_count == 0);
    available_token[2]++;
    remote_wom_poll(); assert(ack_count == 2 && !remote_hid_pending);
    assert(acks[0][4] == 1 && acks[0][5] == 2 && sys_get_le16(&acks[0][6]) == 17);
    assert(acks[1][4] == 2 && acks[1][3] == RCV_HID_ST_OK && acks[1][7] == BIT(2));
    assert(cancelled_mask == BIT(2) && cancelled_token == remote_wom_token);
    unsigned before = sends;
    assert(remote_wom_begin(8, 255, 10, false) == RCV_HID_ST_STARTED);
    assert(submitted == 1 && sends == before && remote_wom_minutes == 10);
    /* Drive the same post-scan state the deferred job publishes. */
    remote_hid_expected = esb_send_remote_wom_command(255, remote_wom_minutes, &remote_wom_token);
    remote_wom_deadline = now + 5000;
    reply(2, 0, EIO); remote_wom_poll();
    assert(ack_count == 3 && sys_get_le16(&acks[2][8]) == EIO && remote_hid_pending);
    now += 5000; remote_wom_poll();
    assert(ack_count == 5 && sys_get_le16(&acks[3][8]) == ETIMEDOUT);
    assert(acks[4][3] == RCV_HID_ST_ENOENT && acks[4][11] == (BIT(2) | BIT(5)));
    assert(!remote_hid_pending);
    assert(rcv_cmd_remote_wom_command(2, 35791) == RCV_HID_ST_STARTED);
    reply(2, 35791, 0); remote_wom_poll();
    assert(ack_count == 5 && !remote_hid_pending); /* console never emits HID */
    request[5] = request[6] = 255;
    assert(hid_wom(request, 3) == RCV_HID_ST_STARTED);
    assert(sent_minutes == ESB_WOM_TIMEOUT_CLEAR);
    on_remote_confirm(2, ESB_PONG_FLAG_WOM_CONTROL);
    remote_wom_poll(); assert(ack_count == 5 && remote_hid_pending);
    reply(2, 0, 0); remote_wom_poll();
    assert(ack_count == 7 && !remote_hid_pending);
    assert(acks[5][4] == RCV_HID_WOM_RESULT && sys_get_le16(&acks[5][6]) == 0 &&
           sys_get_le16(&acks[5][8]) == 0 && acks[6][3] == RCV_HID_ST_OK);
    before = sends;
    assert(remote_wom_begin(8, 255, ESB_WOM_TIMEOUT_CLEAR, false) == RCV_HID_ST_STARTED);
    assert(submitted == 2 && sends == before && remote_wom_minutes == ESB_WOM_TIMEOUT_CLEAR);
    remote_hid_expected = esb_send_remote_wom_command(255, remote_wom_minutes, &remote_wom_token);
    remote_wom_deadline = now + 5000;
    reply(2, 0, 0); reply(5, 0, EIO); remote_wom_poll();
    assert(ack_count == 10 && !remote_hid_pending);
    assert(acks[9][3] == RCV_HID_ST_ENOENT && acks[9][7] == BIT(2) && acks[9][11] == BIT(5));
    assert(rcv_cmd_remote_wom_command(2, ESB_WOM_TIMEOUT_CLEAR) == RCV_HID_ST_STARTED);
    reply(2, 0, 0); remote_wom_poll();
    assert(strstr(console_output, "restored firmware default") && !strstr(console_output, ": 0 minutes"));
    console_used = 0; console_output[0] = 0;
    assert(rcv_cmd_remote_wom_command(2, ESB_WOM_TIMEOUT_CLEAR) == RCV_HID_ST_STARTED);
    reply(2, 0, EIO); remote_wom_poll();
    assert(strstr(console_output, "error 5") && !strstr(console_output, "restored"));
    request[5] = 0xfe; request[6] = 0xff;
    assert(hid_wom(request, 3) == RCV_HID_ST_STARTED && sent_minutes == ESB_WOM_ENTER);
    before = ack_count;
    on_remote_confirm(2, ESB_PONG_FLAG_WOM_CONTROL);
    remote_wom_poll(); assert(ack_count == before && remote_hid_pending);
    reply(2, ESB_WOM_ENTER, 0); available_token[2]--;
    remote_wom_poll(); assert(ack_count == before && remote_hid_pending);
    available_token[2]++;
    remote_wom_poll();
    assert(ack_count == before + 2 && !remote_hid_pending);
    assert(sys_get_le16(&acks[before][6]) == ESB_WOM_ENTER &&
           sys_get_le16(&acks[before][8]) == 0 && acks[before + 1][3] == RCV_HID_ST_OK);
    before = sends;
    assert(remote_wom_begin(8, 255, ESB_WOM_ENTER, false) == RCV_HID_ST_STARTED);
    assert(submitted == 3 && sends == before && remote_wom_minutes == ESB_WOM_ENTER);
    remote_hid_expected = esb_send_remote_wom_command(255, remote_wom_minutes, &remote_wom_token);
    remote_wom_deadline = now + 5000;
    before = ack_count;
    reply(2, ESB_WOM_ENTER, 0); reply(5, 0, EBUSY); remote_wom_poll();
    assert(!remote_hid_pending && ack_count == before + 3);
    assert(acks[before + 2][3] == RCV_HID_ST_ENOENT &&
           acks[before + 2][7] == BIT(2) && acks[before + 2][11] == BIT(5));
    console_used = 0; console_output[0] = 0;
    assert(rcv_cmd_remote_wom_command(2, ESB_WOM_ENTER) == RCV_HID_ST_STARTED);
    reply(2, ESB_WOM_ENTER, 0); remote_wom_poll();
    assert(strstr(console_output, "WOM entry accepted") &&
           !strstr(console_output, "minutes") && !strstr(console_output, "asleep"));
    console_used = 0; console_output[0] = 0;
    assert(rcv_cmd_remote_wom_command(2, ESB_WOM_ENTER) == RCV_HID_ST_STARTED);
    reply(2, 0, ENOTSUP); remote_wom_poll();
    assert(strstr(console_output, "error") && !strstr(console_output, "accepted"));
    request[3] = 1; assert(hid_wom(request, 3) == RCV_HID_ST_EINVAL);
    puts("WOM job: exact token, false ACK, busy, deferred all, errors, timeout, terminal-only");
    return 0;
}
