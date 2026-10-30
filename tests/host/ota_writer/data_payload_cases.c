static uint8_t data_image_byte(uint32_t offset) {
    static const uint8_t header[4] = {0x3d, 0xb8, 0xf3, 0x96};
    if (TEST_MCUBOOT && offset < sizeof(header)) return header[offset];
    return (uint8_t)(offset * 37 + 3);
}

static void data_report(uint16_t seq, size_t available, size_t needed) {
    uint8_t report[64];
    memset(report, 0xa5, sizeof(report));
    report[0] = HID_OTA_DATA;
    report[1] = RECEIVER_OTA_ID;
    sys_put_be16(seq, report + 2);
    for (size_t i = 0; i < MIN(available, needed); i++)
        report[4 + i] = data_image_byte((uint32_t)seq * 60 + i);
    receiver_ota_process_hid(report, 4 + available);
}

struct slot_snapshot {
    struct page_write_req requests[2];
    int sem_count[2];
};

static int slot_sem_count(unsigned slot) {
    struct k_sem *sem = slot_sems[slot];
    pthread_mutex_lock(&sem->lock);
    int count = sem->count;
    pthread_mutex_unlock(&sem->lock);
    return count;
}

static void snapshot_slots(struct slot_snapshot *snapshot) {
    memcpy(snapshot->requests, page_write_reqs, sizeof(snapshot->requests));
    for (unsigned slot = 0; slot < 2; slot++)
        snapshot->sem_count[slot] = slot_sem_count(slot);
}

static void assert_slots_unchanged(const struct slot_snapshot *snapshot) {
    assert(!memcmp(snapshot->requests, page_write_reqs, sizeof(snapshot->requests)));
    for (unsigned slot = 0; slot < 2; slot++)
        assert(snapshot->sem_count[slot] == slot_sem_count(slot));
}

static void data_payload_image(uint32_t size, bool padded) {
    begin(size);
    assert(rcv_ota.state == RCV_OTA_READY);
    uint8_t header_only[4] = {HID_OTA_DATA, RECEIVER_OTA_ID, 0, 0};
    typeof(rcv_ota) initial = rcv_ota;
    struct slot_snapshot snapshot;
    snapshot_slots(&snapshot);
    receiver_ota_process_hid(header_only, 3);
    assert(!memcmp(&initial, &rcv_ota, sizeof(initial)));
    assert_slots_unchanged(&snapshot);

    for (uint16_t seq = 0; (uint32_t)seq * 60 < size; seq++) {
        size_t needed = MIN(60, size - (uint32_t)seq * 60);
        typeof(rcv_ota) before = rcv_ota;
        snapshot_slots(&snapshot);
        data_report(seq, 0, needed);
        assert(!memcmp(&before, &rcv_ota, sizeof(before)));
        assert_slots_unchanged(&snapshot);
        data_report(seq, needed - 1, needed);
        assert(!memcmp(&before, &rcv_ota, sizeof(before)));
        assert_slots_unchanged(&snapshot);
        if (needed == 60) {
            data_report(seq, 1, needed);
            assert(!memcmp(&before, &rcv_ota, sizeof(before)));
            assert_slots_unchanged(&snapshot);
        }

        unsigned slot = rcv_ota.next_write_slot;
        data_report(seq, padded ? 60 : needed, needed);
        assert(rcv_ota.state == RCV_OTA_RECEIVING);
        assert(rcv_ota.next_expected_seq == seq + 1);
        assert(rcv_ota.bytes_written == MIN(size, (uint32_t)(seq + 1) * 60));
        assert(rcv_ota.page_buf_offset == rcv_ota.bytes_written);
        assert(rcv_ota.next_write_slot == slot);
        assert(slot_sem_count(slot) == 0);
        for (uint32_t i = 0; i < rcv_ota.bytes_written; i++)
            assert(page_write_reqs[slot].data[i] == data_image_byte(i));
        assert(!memcmp(page_write_reqs[slot].data + rcv_ota.page_buf_offset,
                       snapshot.requests[slot].data + rcv_ota.page_buf_offset,
                       RCV_OTA_FLASH_PAGE_SIZE - rcv_ota.page_buf_offset));
        assert(page_write_reqs[slot].addr == snapshot.requests[slot].addr);
        assert(page_write_reqs[slot].len == snapshot.requests[slot].len);
        assert(!memcmp(&page_write_reqs[1 - slot], &snapshot.requests[1 - slot],
                       sizeof(page_write_reqs[0])));

        typeof(rcv_ota) accepted = rcv_ota;
        snapshot_slots(&snapshot);
        data_report(seq, 60, needed);
        assert(!memcmp(&accepted, &rcv_ota, sizeof(accepted)));
        assert_slots_unchanged(&snapshot);
        data_report(seq, 0, needed);
        assert(!memcmp(&accepted, &rcv_ota, sizeof(accepted)));
        assert_slots_unchanged(&snapshot);
    }
    assert(rcv_ota.bytes_written == size);
    typeof(rcv_ota) complete = rcv_ota;
    snapshot_slots(&snapshot);
    data_report(rcv_ota.next_expected_seq, 60, 0);
    assert(!memcmp(&complete, &rcv_ota, sizeof(complete)));
    assert_slots_unchanged(&snapshot);
    command(HID_OTA_ABORT);
    assert(rcv_ota.state == RCV_OTA_IDLE);
}

static void data_payload_cases(void) {
    static const uint32_t sizes[] = {1, 59, 60, 61, 121};
    for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
        data_payload_image(sizes[i], false);
        data_payload_image(sizes[i], true);
    }
    begin(121);
    struct slot_snapshot snapshot;
    snapshot_slots(&snapshot);
    data_report(1, 1, 60);
    assert(rcv_ota.state == RCV_OTA_ERROR);
    assert(rcv_ota.error_code == OTA_STATUS_SEQ_ERROR);
    assert(last_status[2] == OTA_STATUS_SEQ_ERROR);
    assert(rcv_ota.next_expected_seq == 0);
    assert(rcv_ota.bytes_written == 0);
    assert(rcv_ota.page_buf_offset == 0);
    assert_slots_unchanged(&snapshot);
    command(HID_OTA_ABORT);

    begin(121);
    data_report(0, 60, 60);
    snapshot_slots(&snapshot);
    data_report(2, 1, 1);
    assert(rcv_ota.state == RCV_OTA_ERROR);
    assert(rcv_ota.error_code == OTA_STATUS_SEQ_ERROR);
    assert(last_status[2] == OTA_STATUS_SEQ_ERROR);
    assert(rcv_ota.next_expected_seq == 1);
    assert(rcv_ota.bytes_written == 60);
    assert(rcv_ota.page_buf_offset == 60);
    assert_slots_unchanged(&snapshot);
    command(HID_OTA_ABORT);
}
