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

static void data_payload_image(uint32_t size, bool padded) {
    begin(size);
    assert(rcv_ota.state == RCV_OTA_READY);
    uint8_t header_only[4] = {HID_OTA_DATA, RECEIVER_OTA_ID, 0, 0};
    typeof(rcv_ota) initial = rcv_ota;
    receiver_ota_process_hid(header_only, 3);
    assert(!memcmp(&initial, &rcv_ota, sizeof(initial)));

    for (uint16_t seq = 0; (uint32_t)seq * 60 < size; seq++) {
        size_t needed = MIN(60, size - (uint32_t)seq * 60);
        typeof(rcv_ota) before = rcv_ota;
        data_report(seq, 0, needed);
        assert(!memcmp(&before, &rcv_ota, sizeof(before)));
        data_report(seq, needed - 1, needed);
        assert(!memcmp(&before, &rcv_ota, sizeof(before)));
        if (needed == 60) {
            data_report(seq, 1, needed);
            assert(!memcmp(&before, &rcv_ota, sizeof(before)));
        }
        data_report(seq, padded ? 60 : needed, needed);
        assert(rcv_ota.state == RCV_OTA_RECEIVING);
        assert(rcv_ota.next_expected_seq == seq + 1);
        assert(rcv_ota.bytes_written == MIN(size, (uint32_t)(seq + 1) * 60));
        assert(rcv_ota.page_buf_offset == rcv_ota.bytes_written);
        for (uint32_t i = 0; i < rcv_ota.bytes_written; i++)
            assert(rcv_ota.page_buf[i] == data_image_byte(i));
        for (size_t i = rcv_ota.page_buf_offset; i < sizeof(rcv_ota.page_buf); i++)
            assert(rcv_ota.page_buf[i] == 0xff);

        typeof(rcv_ota) accepted = rcv_ota;
        data_report(seq, 60, needed);
        assert(!memcmp(&accepted, &rcv_ota, sizeof(accepted)));
        data_report(seq, 0, needed);
        assert(!memcmp(&accepted, &rcv_ota, sizeof(accepted)));
    }
    assert(rcv_ota.bytes_written == size);
    typeof(rcv_ota) complete = rcv_ota;
    data_report(rcv_ota.next_expected_seq, 60, 0);
    assert(!memcmp(&complete, &rcv_ota, sizeof(complete)));
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
    data_report(1, 1, 60);
    assert(rcv_ota.state == RCV_OTA_ERROR);
    assert(rcv_ota.error_code == OTA_STATUS_SEQ_ERROR);
    assert(last_status[2] == OTA_STATUS_SEQ_ERROR);
    assert(rcv_ota.next_expected_seq == 0);
    assert(rcv_ota.bytes_written == 0);
    assert(rcv_ota.page_buf_offset == 0);
    for (size_t i = 0; i < sizeof(rcv_ota.page_buf); i++)
        assert(rcv_ota.page_buf[i] == 0xff);
    command(HID_OTA_ABORT);
}
