#if defined(CONFIG_TDMA_DIAGNOSTICS)
enum timestamp_target { PING_TIMESTAMP, SLOT_TIMESTAMP };
enum preemption_point { VALID_READ, TICKS_READ, TICKS_WRITE, VALID_WRITE };
enum preemption_action { PUBLISH_RADIO, SNAPSHOT_EVENT };

static struct preemption_state {
    enum timestamp_target target;
    enum preemption_point point;
    enum preemption_action action;
    uint32_t radio_ticks;
    bool armed, pending;
    unsigned attempts, deferred, immediate, executions;
} preemption;
static bool observed_valid;
static uint32_t observed_ticks;
static unsigned lock_depth;

static void account_ping_clock(uint8_t id, uint32_t expected, uint32_t current);
static void tdma_check_slot(uint8_t id, uint32_t ticks, int8_t rssi);
static void esb_ack_handler_cb(const uint8_t *data, uint8_t length, uint32_t pipe,
                               struct esb_payload *ack, bool *has_ack, bool *suppress);
static void run_preemption(void);

static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
    assert(lock == &tdma_timestamp_lock && !lock->held && lock_depth == 0);
    lock->held = true;
    ++lock_depth;
    return 0x51;
}

static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    assert(lock == &tdma_timestamp_lock && lock->held && lock_depth == 1 && key == 0x51);
    lock->held = false;
    --lock_depth;
    if (preemption.armed && preemption.pending) {
        run_preemption();
    }
}

static uint32_t snapshot_uptime_ticks(void)
{
    assert(lock_depth == 0);
    return k_uptime_ticks();
}

static void snapshot_log(const char *format, ...)
{
    assert(lock_depth == 0);
    (void)format;
}
#undef LOG_DBG
#undef LOG_WRN
#define LOG_DBG(...) snapshot_log(__VA_ARGS__)
#define LOG_WRN(...) snapshot_log(__VA_ARGS__)

static void attempt_preemption(enum timestamp_target target, enum preemption_point point)
{
    if (!preemption.armed || preemption.target != target || preemption.point != point) {
        return;
    }
    ++preemption.attempts;
    if (lock_depth) {
        ++preemption.deferred;
        preemption.pending = true;
    } else {
        ++preemption.immediate;
        run_preemption();
    }
}

static bool checked_ping_valid(bool valid)
{
    observed_valid = valid;
    attempt_preemption(PING_TIMESTAMP, VALID_READ);
    return valid;
}
static uint32_t checked_ping_ticks(uint32_t ticks)
{
    observed_ticks = ticks;
    attempt_preemption(PING_TIMESTAMP, TICKS_READ);
    return ticks;
}
static bool checked_slot_valid(bool valid)
{
    observed_valid = valid;
    attempt_preemption(SLOT_TIMESTAMP, VALID_READ);
    return valid;
}
static uint32_t checked_slot_ticks(uint32_t ticks)
{
    observed_ticks = ticks;
    attempt_preemption(SLOT_TIMESTAMP, TICKS_READ);
    return ticks;
}
#else
static uint32_t snapshot_uptime_ticks(void) { return k_uptime_ticks(); }
#endif

/* PRODUCTION FUNCTIONS */

#if defined(CONFIG_TDMA_DIAGNOSTICS)
static void publish_radio(void)
{
    uint8_t data[ESB_PING_LEN] = {0};
    uint8_t length = 2;
    data[0] = 1;
    data[1] = 2;
    if (preemption.target == PING_TIMESTAMP) {
        length = ESB_PING_LEN;
        data[0] = ESB_PING_TYPE;
        data[2] = 7;
        data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, data, ESB_PING_LEN - 1);
    }
    struct esb_payload ack = {0};
    bool has_ack = false, suppress = false;
    now_ticks = preemption.radio_ticks;
    tick_context = TICK_RADIO;
    esb_ack_handler_cb(data, length, 3, &ack, &has_ack, &suppress);
    assert(!suppress);
    if (preemption.target == PING_TIMESTAMP) {
        assert(has_ack && ack.pipe == 3 && ack.length == ESB_PONG_LEN);
        assert(ack.data[0] == ESB_PONG_TYPE && ack.data[1] == 2 && ack.data[2] == 7);
        assert(sys_get_be32(&ack.data[3]) == preemption.radio_ticks);
        assert(crc8_ccitt(0x07, ack.data, ESB_PONG_LEN - 1) == ack.data[ESB_PONG_LEN - 1]);
    } else {
        assert(!has_ack);
    }
    tick_context = TICK_EVENT;
}

static void snapshot_event(void)
{
    if (preemption.target == PING_TIMESTAMP) {
        account_ping_clock(2, 132790, 132795);
    } else {
        tdma_check_slot(2, 13, 40);
    }
}

static void run_preemption(void)
{
    assert(preemption.armed && lock_depth == 0);
    preemption.armed = false;
    preemption.pending = false;
    ++preemption.executions;
    if (preemption.action == PUBLISH_RADIO) {
        publish_radio();
    } else {
        snapshot_event();
    }
}

static void prepare(enum timestamp_target target, uint32_t ticks, bool valid,
                    uint32_t radio_ticks, enum preemption_point point,
                    enum preemption_action action)
{
    assert(lock_depth == 0 && !preemption.armed && !preemption.pending);
    memset(&tracker_diags[2], 0, sizeof(tracker_diags[2]));
    memset(&tracker_slots[2], 0, sizeof(tracker_slots[2]));
    tracker_slots[2].tdma_config_packed = (1u << 16) | (20u << 8);
    tracker_diags[3].g_ping_isr_rx_ticks = 0x12345678;
    tracker_diags[3].g_ping_isr_rx_ticks_valid = true;
    tracker_diags[3].g_last_isr_rx_ticks = 0x87654321;
    tracker_diags[3].g_last_isr_rx_valid = true;
    if (target == PING_TIMESTAMP) {
        tracker_diags[2].g_ping_isr_rx_ticks = ticks;
        tracker_diags[2].g_ping_isr_rx_ticks_valid = valid;
    } else {
        tracker_diags[2].g_last_isr_rx_ticks = ticks;
        tracker_diags[2].g_last_isr_rx_valid = valid;
    }
    preemption = (struct preemption_state){
        .target = target, .point = point, .action = action,
        .radio_ticks = radio_ticks, .armed = true,
    };
    observed_valid = false;
    observed_ticks = 0;
}

static void check_preemption(bool valid, uint32_t ticks)
{
    assert(!preemption.armed && !preemption.pending && lock_depth == 0);
    assert(preemption.attempts == 1 && preemption.deferred == 1);
    assert(preemption.immediate == 0 && preemption.executions == 1);
    assert(observed_valid == valid && observed_ticks == ticks);
    assert(tracker_diags[3].g_ping_isr_rx_ticks == 0x12345678);
    assert(tracker_diags[3].g_ping_isr_rx_ticks_valid);
    assert(tracker_diags[3].g_last_isr_rx_ticks == 0x87654321);
    assert(tracker_diags[3].g_last_isr_rx_valid);
}

static void test_reader_preemption(enum preemption_point point)
{
    prepare(PING_TIMESTAMP, 132768, true, 900, point, PUBLISH_RADIO);
    account_ping_clock(2, 132754, 132795);
    check_preemption(true, 132768);
    assert(tracker_diags[2].g_last_ping_isr_rx_ticks_raw == 132768);
    assert(tracker_diags[2].g_last_ping_rx_time_diff_valid);
    assert(tracker_diags[2].g_last_ping_rx_time_diff_ticks == 9);
    assert(tracker_diags[2].g_ping_isr_rx_ticks_valid);
    assert(tracker_diags[2].g_ping_isr_rx_ticks == 900);

    prepare(PING_TIMESTAMP, 132768, false, 132800, point, PUBLISH_RADIO);
    account_ping_clock(2, 132780, 132795);
    check_preemption(false, 132768);
    assert(tracker_diags[2].g_last_ping_isr_rx_ticks_raw == 132795);
    assert(tracker_diags[2].g_last_ping_rx_time_diff_valid);
    assert(tracker_diags[2].g_last_ping_rx_time_diff_ticks == 10);
    assert(tracker_diags[2].g_ping_isr_rx_ticks_valid);
    assert(tracker_diags[2].g_ping_isr_rx_ticks == 132800);

    prepare(PING_TIMESTAMP, 2, true, 12, point, PUBLISH_RADIO);
    account_ping_clock(2, UINT32_MAX - 2u, 20);
    check_preemption(true, 2);
    assert(tracker_diags[2].g_last_ping_isr_rx_ticks_raw == 2);
    assert(tracker_diags[2].g_last_ping_rx_time_diff_ticks == 0);
    assert(tracker_diags[2].g_last_ping_rx_time_diff_valid);
    assert(tracker_diags[2].g_ping_isr_rx_ticks_valid);
    assert(tracker_diags[2].g_ping_isr_rx_ticks == 12);

    prepare(SLOT_TIMESTAMP, 12, true, 17, point, PUBLISH_RADIO);
    tdma_check_slot(2, 13, 40);
    check_preemption(true, 12);
    assert(tracker_diags[2].stats.count == 1);
    assert(tracker_diags[2].stats.raw_min_offset == 2);
    assert(tracker_diags[2].stats.phase_hist[66] == 1);
    assert(tracker_diags[2].stats.rssi_hist[40] == 1);
    assert(tracker_diags[2].g_last_isr_rx_valid);
    assert(tracker_diags[2].g_last_isr_rx_ticks == 17);

    prepare(SLOT_TIMESTAMP, 12, false, 17, point, PUBLISH_RADIO);
    tdma_check_slot(2, 13, 40);
    check_preemption(false, 12);
    assert(tracker_diags[2].stats.raw_min_offset == 3);
    assert(tracker_diags[2].stats.phase_hist[67] == 1);
    assert(tracker_diags[2].g_last_isr_rx_valid);
    assert(tracker_diags[2].g_last_isr_rx_ticks == 17);

    prepare(SLOT_TIMESTAMP, UINT32_MAX - 1u, true, 15, point, PUBLISH_RADIO);
    tdma_check_slot(2, 16, 40);
    check_preemption(true, UINT32_MAX - 1u);
    assert(tracker_diags[2].stats.raw_min_offset == 4);
    assert(tracker_diags[2].stats.phase_hist[68] == 1);
    assert(tracker_diags[2].g_last_isr_rx_valid);
    assert(tracker_diags[2].g_last_isr_rx_ticks == 15);
}

static void test_writer_preemption(enum preemption_point point)
{
    /* An EVENT observer during RADIO is a lock stress model, not the IRQ priority order. */
    prepare(PING_TIMESTAMP, 132768, false, 132800, point, SNAPSHOT_EVENT);
    publish_radio();
    check_preemption(true, 132800);
    assert(tracker_diags[2].g_last_ping_isr_rx_ticks_raw == 132800);
    assert(tracker_diags[2].g_last_ping_rx_time_diff_ticks == 5);

    prepare(SLOT_TIMESTAMP, 12, false, 17, point, SNAPSHOT_EVENT);
    publish_radio();
    check_preemption(true, 17);
    assert(tracker_diags[2].stats.count == 1);
    assert(tracker_diags[2].stats.raw_min_offset == 7);
    assert(tracker_diags[2].stats.phase_hist[71] == 1);
}
#endif

int main(void)
{
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    test_reader_preemption(VALID_READ);
    test_reader_preemption(TICKS_READ);
    test_writer_preemption(TICKS_WRITE);
    test_writer_preemption(VALID_WRITE);
    puts("diagnostic timestamp publication and snapshots passed");
#else
    puts("diagnostic timestamp snapshots disabled");
#endif
    return 0;
}
