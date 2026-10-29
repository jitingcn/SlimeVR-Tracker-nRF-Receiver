static bool crc_settings_case(const char *name) {
    unsigned failure;
    if(!strcmp(name,"crc_settings_first")) failure=1;
    else if(!strcmp(name,"crc_settings_later")) failure=3;
    else if(!strcmp(name,"crc_settings_ok")) failure=0;
    else return false;
#if TEST_UF2
    begin(521);
    assert(rcv_ota.state==RCV_OTA_READY);
    for(size_t i=0;i<512;i++) staging[i]=(uint8_t)i;
    memcpy(staging+512,"123456789",9);
    read_base=rcv_ota.staging_base;
    fail_read_call=failure;
    memset(&prepared_bl_settings,0xa5,sizeof(prepared_bl_settings));
    struct bootloader_settings unpublished=prepared_bl_settings;
    rcv_ota.state=RCV_OTA_VERIFYING;
    rcv_ota.error_code=OTA_STATUS_VERIFY_OK;
    command(HID_OTA_ACTIVATE);
    pthread_mutex_lock(&gate_lock);
    while(!thread_exited) pthread_cond_wait(&gate_cond,&gate_lock);
    pthread_mutex_unlock(&gate_lock);
    assert(reads==(failure?failure:3));
    if(failure) {
        assert(rcv_ota.state==RCV_OTA_ERROR);
        assert(rcv_ota.error_code==OTA_STATUS_FLASH_ERROR);
        assert(last_status[0]==HID_OTA_STATUS && last_status[1]==RECEIVER_OTA_ID);
        assert(last_status[2]==OTA_STATUS_FLASH_ERROR);
        assert(!bl_settings_prepared && copies==0 && reboots==0);
        assert(!memcmp(&prepared_bl_settings,&unpublished,sizeof(unpublished)));
    } else {
        const struct bootloader_settings expected={
            .bank_0=0x01,
            .bank_0_crc=0x8944,
            .bank_1=0xff,
            .bank_0_size=521,
        };
        assert(bl_settings_prepared && copies==1 && reboots==0);
        assert(!memcmp(&prepared_bl_settings,&expected,sizeof(expected)));
    }
    command(HID_OTA_ABORT);
    assert(rcv_ota_writer_thread.joined);
    printf("PASS %s uf2=%d\n",name,TEST_UF2);
    return true;
#else
    (void)failure;
    abort();
#endif
}
