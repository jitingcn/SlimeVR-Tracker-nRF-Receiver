/* Literal image and CRC oracles for the data_report input, independent of the
 * firmware CRC implementation. The MCUboot image replaces the first four bytes. */
static const uint8_t slot_expected_block[256] = {
    0x03, 0x28, 0x4d, 0x72, 0x97, 0xbc, 0xe1, 0x06, 0x2b, 0x50, 0x75, 0x9a, 0xbf, 0xe4, 0x09, 0x2e,
    0x53, 0x78, 0x9d, 0xc2, 0xe7, 0x0c, 0x31, 0x56, 0x7b, 0xa0, 0xc5, 0xea, 0x0f, 0x34, 0x59, 0x7e,
    0xa3, 0xc8, 0xed, 0x12, 0x37, 0x5c, 0x81, 0xa6, 0xcb, 0xf0, 0x15, 0x3a, 0x5f, 0x84, 0xa9, 0xce,
    0xf3, 0x18, 0x3d, 0x62, 0x87, 0xac, 0xd1, 0xf6, 0x1b, 0x40, 0x65, 0x8a, 0xaf, 0xd4, 0xf9, 0x1e,
    0x43, 0x68, 0x8d, 0xb2, 0xd7, 0xfc, 0x21, 0x46, 0x6b, 0x90, 0xb5, 0xda, 0xff, 0x24, 0x49, 0x6e,
    0x93, 0xb8, 0xdd, 0x02, 0x27, 0x4c, 0x71, 0x96, 0xbb, 0xe0, 0x05, 0x2a, 0x4f, 0x74, 0x99, 0xbe,
    0xe3, 0x08, 0x2d, 0x52, 0x77, 0x9c, 0xc1, 0xe6, 0x0b, 0x30, 0x55, 0x7a, 0x9f, 0xc4, 0xe9, 0x0e,
    0x33, 0x58, 0x7d, 0xa2, 0xc7, 0xec, 0x11, 0x36, 0x5b, 0x80, 0xa5, 0xca, 0xef, 0x14, 0x39, 0x5e,
    0x83, 0xa8, 0xcd, 0xf2, 0x17, 0x3c, 0x61, 0x86, 0xab, 0xd0, 0xf5, 0x1a, 0x3f, 0x64, 0x89, 0xae,
    0xd3, 0xf8, 0x1d, 0x42, 0x67, 0x8c, 0xb1, 0xd6, 0xfb, 0x20, 0x45, 0x6a, 0x8f, 0xb4, 0xd9, 0xfe,
    0x23, 0x48, 0x6d, 0x92, 0xb7, 0xdc, 0x01, 0x26, 0x4b, 0x70, 0x95, 0xba, 0xdf, 0x04, 0x29, 0x4e,
    0x73, 0x98, 0xbd, 0xe2, 0x07, 0x2c, 0x51, 0x76, 0x9b, 0xc0, 0xe5, 0x0a, 0x2f, 0x54, 0x79, 0x9e,
    0xc3, 0xe8, 0x0d, 0x32, 0x57, 0x7c, 0xa1, 0xc6, 0xeb, 0x10, 0x35, 0x5a, 0x7f, 0xa4, 0xc9, 0xee,
    0x13, 0x38, 0x5d, 0x82, 0xa7, 0xcc, 0xf1, 0x16, 0x3b, 0x60, 0x85, 0xaa, 0xcf, 0xf4, 0x19, 0x3e,
    0x63, 0x88, 0xad, 0xd2, 0xf7, 0x1c, 0x41, 0x66, 0x8b, 0xb0, 0xd5, 0xfa, 0x1f, 0x44, 0x69, 0x8e,
    0xb3, 0xd8, 0xfd, 0x22, 0x47, 0x6c, 0x91, 0xb6, 0xdb, 0x00, 0x25, 0x4a, 0x6f, 0x94, 0xb9, 0xde,
};

#define SLOT_CRC_4096  (TEST_MCUBOOT ? 0x636d19cau : 0xdc3f03c5u)
#define SLOT_CRC_4097  (TEST_MCUBOOT ? 0x30d9f880u : 0xa0d9b70bu)
#define SLOT_CRC_4140  (TEST_MCUBOOT ? 0x4e7f4b06u : 0x28cc296au)
#define SLOT_CRC_8193  (TEST_MCUBOOT ? 0xa3d1925eu : 0x96fd6ff7u)
#define SLOT_CRC_12289 (TEST_MCUBOOT ? 0x03ce7806u : 0xa0621bbfu)

static void slot_expect_status(uint8_t status, uint16_t seq, uint32_t bytes)
{
    assert(last_status[0] == HID_OTA_STATUS);
    assert(last_status[1] == RECEIVER_OTA_ID);
    assert(last_status[2] == status);
    assert(sys_get_be16(last_status + 3) == seq);
    assert(sys_get_le32(last_status + 5) == bytes);
}

static void slot_expect_count(unsigned slot, int count)
{
    struct k_sem *sem = slot_sems[slot];
    pthread_mutex_lock(&sem->lock);
    assert(sem->count == count);
    pthread_mutex_unlock(&sem->lock);
}

static void slot_expect_bytes(const uint8_t *actual, uint32_t offset, size_t len)
{
    static const uint8_t mcuboot_header[] = {0x3d, 0xb8, 0xf3, 0x96};
    for (size_t i = 0; i < len; i++) {
        uint32_t pos = offset + i;
        uint8_t expected = TEST_MCUBOOT && pos < sizeof(mcuboot_header)
            ? mcuboot_header[pos] : slot_expected_block[pos % sizeof(slot_expected_block)];
        assert(actual[i] == expected);
    }
}

static void slot_expect_image(uint32_t size)
{
    slot_expect_bytes(staging, 0, size);
    if (size % RCV_OTA_FLASH_PAGE_SIZE) {
        uint32_t end = (size + RCV_OTA_FLASH_PAGE_SIZE - 1) & ~(RCV_OTA_FLASH_PAGE_SIZE - 1);
        for (uint32_t i = size; i < end; i++) assert(staging[i] == 0xff);
    }
}

static void slot_begin_checked(uint32_t size, uint32_t crc)
{
    begin_with_crc(size, crc);
    slot_expect_status(OTA_STATUS_READY, 0, 0);
}

static void slot_send_range(uint16_t first, uint16_t end, uint32_t size, bool padded)
{
    for (uint16_t seq = first; seq < end; seq++) {
        size_t needed = MIN(60, size - (uint32_t)seq * 60);
        data_report(seq, padded ? 60 : needed, needed);
    }
}

static int slot_verify_result;
static void slot_verify_observer(const uint8_t *status)
{
    pthread_mutex_lock(&gate_lock);
    slot_verify_result = status[2];
    pthread_cond_broadcast(&gate_cond);
    pthread_mutex_unlock(&gate_lock);
}

static void slot_start_verify(void)
{
    pthread_mutex_lock(&gate_lock);
    slot_verify_result = -1;
    pthread_mutex_unlock(&gate_lock);
    status_observer = slot_verify_observer;
    command(HID_OTA_VERIFY);
}

static void slot_wait_verify(uint16_t seq, uint32_t size)
{
    pthread_mutex_lock(&gate_lock);
    while (slot_verify_result < 0) pthread_cond_wait(&gate_cond, &gate_lock);
    assert(slot_verify_result == OTA_STATUS_VERIFY_OK);
    pthread_mutex_unlock(&gate_lock);
    wait_writer_idle();
    status_observer = NULL;
    slot_expect_status(OTA_STATUS_VERIFY_OK, seq, size);
}

static void slot_boundary(void)
{
    slot_begin_checked(4140, SLOT_CRC_4140);
    hold_write = true;
    slot_send_range(0, 68, 4140, false);
    assert(rcv_ota.page_buf_offset == 4080);
    data_report(68, 60, 60);
    wait_leaf();

    assert(rcv_ota.next_write_slot == 1 && rcv_ota.page_buf_offset == 44);
    slot_expect_count(0, 0);
    slot_expect_count(1, 0);
    assert(!memcmp(page_write_reqs[0].data + 4080, slot_expected_block + 240, 16));
    assert(!memcmp(page_write_reqs[1].data, slot_expected_block, 44));
    slot_expect_bytes(page_write_reqs[0].data, 0, 4096);

    release_leaf_gate();
    wait_slot_free(0);
    slot_start_verify();
    slot_wait_verify(69, 4140);
    assert(writes == 2);
    slot_expect_image(4140);
    command(HID_OTA_ABORT);
}

static void slot_reuse_crc(void)
{
    slot_begin_checked(12289, SLOT_CRC_12289);
    hold_write = true;
    slot_send_range(0, 69, 12289, false);
    wait_leaf();
    assert(rcv_ota.page_buf_offset == 44 && rcv_ota.next_write_slot == 1);
    release_leaf_gate();
    wait_slot_free(0);

    slot_send_range(69, 137, 12289, false);
    assert(rcv_ota.page_buf_offset == 28 && rcv_ota.next_write_slot == 0);
    wait_slot_free(1);
    slot_send_range(137, 205, 12289, false);
    assert(rcv_ota.page_buf_offset == 1 && rcv_ota.next_write_slot == 1);
    wait_slot_free(0);

    slot_start_verify();
    slot_wait_verify(205, 12289);
    assert(writes == 4);
    slot_expect_image(12289);
    assert(staging[12288] == 0x03 && staging[12289] == 0xff);
    command(HID_OTA_ABORT);
}

static void slot_tail(bool padded)
{
    slot_begin_checked(4097, SLOT_CRC_4097);
    hold_write = true;
    slot_send_range(0, 68, 4097, false);
    data_report(68, padded ? 60 : 17, 17);
    wait_leaf();
    assert(rcv_ota.page_buf_offset == 1 && rcv_ota.next_write_slot == 1);
    assert(page_write_reqs[1].data[0] == 0x03);
    slot_expect_count(0, 0);
    slot_expect_count(1, 0);

    slot_start_verify();
    assert(reads == 0 && writes == 0);
    release_leaf_gate();
    slot_wait_verify(69, 4097);
    assert(writes == 2);
    slot_expect_image(4097);
    assert(staging[4095] == 0xde && staging[4096] == 0x03 && staging[4097] == 0xff);
    command(HID_OTA_ABORT);
}

static void slot_exact_page(void)
{
    slot_begin_checked(4096, SLOT_CRC_4096);
    slot_send_range(0, 69, 4096, false);
    assert(rcv_ota.page_buf_offset == 0 && rcv_ota.next_write_slot == 1);
    wait_slot_free(0);
    slot_start_verify();
    slot_wait_verify(69, 4096);
    assert(writes == 1);
    slot_expect_image(4096);
    assert(staging[4095] == 0xde);
    command(HID_OTA_ABORT);
}

static void slot_blocked(void)
{
    slot_begin_checked(4096, SLOT_CRC_4096);
    memset(page_write_reqs[0].data, 0x6c, sizeof(page_write_reqs[0].data));
    memset(page_write_reqs[1].data, 0x93, sizeof(page_write_reqs[1].data));
    fail_sem_take = 1;
    data_report(0, 60, 60);
    slot_expect_status(OTA_STATUS_FLASH_ERROR, 0, 0);
    assert(rcv_ota.page_buf_offset == 0);
    slot_expect_count(0, 1);
    slot_expect_count(1, 1);
    for (size_t i = 0; i < 4096; i++) {
        assert(page_write_reqs[0].data[i] == 0x6c);
        assert(page_write_reqs[1].data[i] == 0x93);
    }
    command(HID_OTA_ABORT);

    slot_begin_checked(12289, SLOT_CRC_12289);
    hold_write = true;
    slot_send_range(0, 69, 12289, false);
    wait_leaf();
    slot_send_range(69, 136, 12289, false);
    assert(rcv_ota.page_buf_offset == 4064);
    uint8_t first[4096];
    memcpy(first, page_write_reqs[0].data, sizeof(first));
    data_report(136, 60, 60);
    slot_expect_status(OTA_STATUS_FLASH_ERROR, 136, 8160);
    assert(rcv_ota.page_buf_offset == 0 && rcv_ota.next_write_slot == 0);
    slot_expect_count(0, 0);
    slot_expect_count(1, 0);
    assert(!memcmp(first, page_write_reqs[0].data, sizeof(first)));
    slot_expect_bytes(page_write_reqs[0].data, 0, 4096);
    slot_expect_bytes(page_write_reqs[1].data, 4096, 4096);
    uint8_t second[4096];
    memcpy(second, page_write_reqs[1].data, sizeof(second));
    data_report(136, 60, 60);
    command(HID_OTA_VERIFY);
    slot_expect_status(OTA_STATUS_FLASH_ERROR, 136, 8160);
    assert(!memcmp(first, page_write_reqs[0].data, sizeof(first)));
    assert(!memcmp(second, page_write_reqs[1].data, sizeof(second)));

    release_leaf_gate();
    wait_slot_free(0);
    wait_slot_free(1);
    assert(writes == 2);
    slot_expect_bytes(staging, 0, 8192);
    command(HID_OTA_ABORT);
}

static void slot_invalid_begin(void)
{
    uint8_t report[64] = {HID_OTA_BEGIN, RECEIVER_OTA_ID};
    sys_put_le32(4096, report + 2);
    sys_put_le32(SLOT_CRC_4096, report + 6);
    sys_put_be16(69, report + 10);
    report[12] = OTA_PROTOCOL_VERSION + 1;
    memcpy(report + 13, CONFIG_BOARD, sizeof(CONFIG_BOARD));
    receiver_ota_process_hid(report, sizeof(report));
}

static void slot_queue_failure(void)
{
    slot_begin_checked(8193, SLOT_CRC_8193);
    memset(page_write_reqs[1].data, 0x6c, sizeof(page_write_reqs[1].data));
    slot_send_range(0, 68, 8193, false);
    fail_command = 0;
    data_report(68, 60, 60);
    slot_expect_status(OTA_STATUS_FLASH_ERROR, 68, 4080);
    assert(rcv_ota.page_buf_offset == 4096);
    assert(rcv_ota.next_write_slot == 0 && rcv_ota.page_buf_flash_addr == rcv_ota.staging_base);
    slot_expect_count(0, 0);
    slot_expect_count(1, 1);
    slot_expect_bytes(page_write_reqs[0].data, 0, 4096);
    for (size_t i = 0; i < 4096; i++) assert(page_write_reqs[1].data[i] == 0x6c);
    data_report(68, 60, 60);
    command(HID_OTA_VERIFY);
    slot_expect_status(OTA_STATUS_FLASH_ERROR, 68, 4080);
    assert(rcv_ota.page_buf_offset == 4096);

    join_failures = 1;
    command(HID_OTA_ABORT);
    slot_expect_status(OTA_STATUS_TIMEOUT, 68, 4080);
    assert(rcv_ota.page_buf_offset == 4096);
    slot_expect_count(0, 0);
    unsigned old_erases = erases;
    slot_invalid_begin();
    slot_expect_status(OTA_STATUS_ERROR, 68, 4080);
    assert(rcv_ota.page_buf_offset == 0);
    assert(!writer_created && erases == old_erases);
    slot_expect_count(0, 1);
    slot_expect_bytes(page_write_reqs[0].data, 0, 4096);

    fail_command = -1;
    slot_begin_checked(4097, SLOT_CRC_4097);
    slot_send_range(0, 69, 4097, false);
    wait_slot_free(0);
    fail_command = 1;
    command(HID_OTA_VERIFY);
    slot_expect_status(OTA_STATUS_FLASH_ERROR, 69, 4097);
    assert(rcv_ota.page_buf_offset == 1);
    assert(rcv_ota.next_write_slot == 1);
    slot_expect_count(1, 0);
    assert(page_write_reqs[1].data[0] == 0x03);
    for (size_t i = 1; i < 4096; i++) assert(page_write_reqs[1].data[i] == 0xff);
    fail_command = -1;
    command(HID_OTA_ABORT);
    assert(!receiver_ota_is_active());
    slot_expect_count(1, 1);
}

static void slot_flash_failure(bool erase_failure)
{
    slot_begin_checked(8193, SLOT_CRC_8193);
    if (erase_failure) {
        hold_erase = true;
        fail_erase_call = 1;
    } else {
        hold_write = true;
        fail_write_call = 1;
    }
    slot_send_range(0, 69, 8193, false);
    wait_leaf();
    assert(rcv_ota.page_buf_offset == 44 && rcv_ota.next_write_slot == 1);
    slot_expect_count(0, 0);
    slot_expect_count(1, 0);
    uint8_t partial[44];
    memcpy(partial, page_write_reqs[1].data, sizeof(partial));
    slot_expect_bytes(partial, 4096, sizeof(partial));

    release_leaf_gate();
    wait_slot_free(0);
    assert(atomic_get(&writer_error) == 1 && writes == 0);
    slot_expect_count(1, 0);
    data_report(69, 60, 60);
    slot_expect_status(OTA_STATUS_FLASH_ERROR, 69, 4140);
    assert(rcv_ota.page_buf_offset == 44);
    assert(!memcmp(partial, page_write_reqs[1].data, sizeof(partial)));
    if (!erase_failure) {
        for (size_t i = 0; i < 4096; i++) assert(staging[i] == 0xff);
    }
    command(HID_OTA_VERIFY);
    slot_expect_status(OTA_STATUS_FLASH_ERROR, 69, 4140);
    slot_expect_count(1, 0);
    command(HID_OTA_ABORT);
    assert(!receiver_ota_is_active());
    slot_expect_count(1, 1);
}

static void slot_abort_partial(void)
{
    slot_begin_checked(4096, SLOT_CRC_4096);
    uint8_t before[60];
    memcpy(before, staging, sizeof(before));
    data_report(0, 60, 60);
    assert(receiver_ota_is_active() && rcv_ota.page_buf_offset == 60);
    slot_expect_count(0, 0);
    command(HID_OTA_ABORT);
    assert(!receiver_ota_is_active() && writes == 0);
    assert(!memcmp(before, staging, sizeof(before)));
    slot_expect_count(0, 1);

    slot_begin_checked(4096, SLOT_CRC_4096);
    slot_send_range(0, 69, 4096, false);
    wait_slot_free(0);
    slot_start_verify();
    slot_wait_verify(69, 4096);
    assert(writes == 1);
    slot_expect_image(4096);
    command(HID_OTA_ABORT);
}

static void slot_join_timeout(void)
{
    slot_begin_checked(8193, SLOT_CRC_8193);
    hold_write = true;
    slot_send_range(0, 69, 8193, false);
    wait_leaf();
    assert(rcv_ota.page_buf_offset == 44 && rcv_ota.next_write_slot == 1);
    typeof(rcv_ota) saved = rcv_ota;
    uint8_t first[4096], partial[44];
    memcpy(first, page_write_reqs[0].data, sizeof(first));
    memcpy(partial, page_write_reqs[1].data, sizeof(partial));
    slot_expect_count(0, 0);
    slot_expect_count(1, 0);

    join_failures = 2;
    command(HID_OTA_ABORT);
    slot_expect_status(OTA_STATUS_TIMEOUT, 69, 4140);
    assert(!memcmp(&saved, &rcv_ota, sizeof(saved)));
    data_report(69, 60, 60);
    command(HID_OTA_VERIFY);
    slot_expect_status(OTA_STATUS_TIMEOUT, 69, 4140);
    begin_with_crc(4096, SLOT_CRC_4096);
    slot_expect_status(OTA_STATUS_TIMEOUT, 69, 4140);
    assert(!memcmp(&saved, &rcv_ota, sizeof(saved)));
    assert(!memcmp(first, page_write_reqs[0].data, sizeof(first)));
    assert(!memcmp(partial, page_write_reqs[1].data, sizeof(partial)));
    slot_expect_count(0, 0);
    slot_expect_count(1, 0);

    release_leaf_gate();
    wait_for_exit();
    slot_expect_count(0, 1);
    slot_expect_count(1, 0);
    slot_expect_bytes(staging, 0, 4096);
    slot_begin_checked(4096, SLOT_CRC_4096);
    slot_expect_count(0, 1);
    slot_expect_count(1, 1);
    slot_send_range(0, 69, 4096, false);
    wait_slot_free(0);
    slot_start_verify();
    slot_wait_verify(69, 4096);
    slot_expect_image(4096);
    command(HID_OTA_ABORT);
}

static bool slot_immediate_done;
static void slot_complete_before_put_returns(uint8_t cmd)
{
    if (cmd != 0) return;
    assert(!slot_immediate_done);
    wait_slot_free(0);
    assert(k_sem_take(slot_sems[0], K_NO_WAIT) == 0);
    memset(page_write_reqs[0].data, 0xc7, sizeof(page_write_reqs[0].data));
    page_write_reqs[0].addr = 0x13579bdf;
    page_write_reqs[0].len = 1234;
    k_sem_give(slot_sems[0]);
    slot_immediate_done = true;
}

static void slot_immediate(void)
{
    slot_begin_checked(4140, SLOT_CRC_4140);
    slot_send_range(0, 68, 4140, false);
    after_put = slot_complete_before_put_returns;
    data_report(68, 60, 60);
    after_put = NULL;
    assert(slot_immediate_done && writes == 1);
    assert(rcv_ota.next_write_slot == 1 && rcv_ota.page_buf_offset == 44);
    assert(page_write_reqs[0].addr == 0x13579bdf && page_write_reqs[0].len == 1234);
    for (size_t i = 0; i < 4096; i++) assert(page_write_reqs[0].data[i] == 0xc7);
    slot_expect_bytes(page_write_reqs[1].data, 4096, 44);
    slot_expect_bytes(staging, 0, 4096);
    slot_expect_count(0, 1);
    slot_expect_count(1, 0);
    slot_start_verify();
    slot_wait_verify(69, 4140);
    slot_expect_image(4140);
    command(HID_OTA_ABORT);
}

static void slot_verify_fifo(void)
{
    slot_begin_checked(4097, SLOT_CRC_4097);
    hold_write = true;
    slot_send_range(0, 69, 4097, false);
    wait_leaf();
    slot_start_verify();
    assert(writes == 0 && reads == 0);
    release_leaf_gate();
    slot_wait_verify(69, 4097);
    assert(writes == 2 && reads == 17);
    slot_expect_image(4097);
    command(HID_OTA_ABORT);
}

static bool slot_case(const char *name)
{
    if (!strcmp(name, "slot_boundary")) slot_boundary();
    else if (!strcmp(name, "slot_reuse_crc")) slot_reuse_crc();
    else if (!strcmp(name, "slot_tail_short")) slot_tail(false);
    else if (!strcmp(name, "slot_tail_padded")) slot_tail(true);
    else if (!strcmp(name, "slot_exact_page")) slot_exact_page();
    else if (!strcmp(name, "slot_blocked")) slot_blocked();
    else if (!strcmp(name, "slot_queue_failure")) slot_queue_failure();
    else if (!strcmp(name, "slot_erase_failure")) slot_flash_failure(true);
    else if (!strcmp(name, "slot_write_failure")) slot_flash_failure(false);
    else if (!strcmp(name, "slot_abort_partial")) slot_abort_partial();
    else if (!strcmp(name, "slot_join_timeout")) slot_join_timeout();
    else if (!strcmp(name, "slot_immediate")) slot_immediate();
    else if (!strcmp(name, "slot_verify_fifo")) slot_verify_fifo();
    else return false;
    return true;
}
