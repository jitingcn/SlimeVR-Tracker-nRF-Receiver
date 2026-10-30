#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include "rcv_cmd.h"
#include "console_send.h"
#include "parse_args.h"
#include "console_u8.inc"

static unsigned requests;
static uint8_t last_target, last_flag, last_axis;
static uint16_t last_value;
uint8_t rcv_cmd_remote_flag(uint8_t target, uint8_t flag)
{
    assert(rcv_hid_opcode_is_pong_flag(flag));
    requests++;
    last_target = target;
    last_flag = flag;
    return RCV_HID_ST_QUEUED;
}
uint8_t rcv_cmd_tracker_channel_all(uint8_t channel)
{
    requests++;
    last_value = channel;
    return RCV_HID_ST_STARTED;
}
uint8_t rcv_cmd_tracker_channel_clear_all(void) { assert(false); return 0; }
uint8_t rcv_cmd_remote_sens_set(uint8_t target, float x, float y, float z) { assert(false); return 0; }
uint8_t rcv_cmd_remote_sens_auto(uint8_t target, uint8_t axis, uint16_t rev)
{
    requests++;
    last_target = target;
    last_axis = axis;
    last_value = rev;
    return RCV_HID_ST_QUEUED;
}
uint8_t rcv_cmd_remote_test_on(uint8_t target, uint16_t tps)
{
    requests++;
    last_target = target;
    last_value = tps;
    return RCV_HID_ST_STARTED;
}
uint8_t rcv_cmd_remote_test_off(uint8_t target) { assert(false); return 0; }
static uint16_t last_minutes;
uint8_t rcv_cmd_remote_wom_command(uint8_t target, uint16_t minutes)
{
    requests++;
    last_target = target;
    last_minutes = minutes;
    return RCV_HID_ST_STARTED;
}

static void test_wom_commands(void)
{
    char *targets[] = {"0", "all"};
    char *invalid[] = {"", "-0", "-1", "+1", " 1", "1 ", "1.5", "1m", "0+1",
                       "35792", "65534", "65535", "99999999999999999999"};
    for (unsigned i = 0; i < 2; i++) {
        requests = 0;
        console_handle_send(targets[i], "wom", "timeout", NULL, NULL);
        assert(requests == 1 && last_minutes == 0 && last_target == (i ? 255 : 0));
        console_handle_send(targets[i], "wom", "timeout", "1", NULL);
        assert(requests == 2 && last_minutes == 1);
        console_handle_send(targets[i], "wom", "timeout", "35791", NULL);
        assert(requests == 3 && last_minutes == 35791);
        console_handle_send(targets[i], "wom", "timeout", "00035791", NULL);
        assert(requests == 4 && last_minutes == 35791);
        console_handle_send(targets[i], "wom", "timeout", "0", NULL);
        assert(requests == 5 && last_minutes == ESB_WOM_TIMEOUT_CLEAR);
        for (unsigned j = 0; j < sizeof(invalid) / sizeof(invalid[0]); j++) {
            console_handle_send(targets[i], "wom", "timeout", invalid[j], NULL);
        }
        console_handle_send(targets[i], "wom", NULL, NULL, NULL);
        console_handle_send(targets[i], "wom", "off", NULL, NULL);
        console_handle_send(targets[i], "wom", "timeout", "1", "extra");
        assert(requests == 5);
        console_handle_send(targets[i], "wom", "enter", NULL, NULL);
        assert(requests == 6 && last_minutes == ESB_WOM_ENTER && last_target == (i ? 255 : 0));
        console_handle_send(targets[i], "wom", "enter", "0", NULL);
        console_handle_send(targets[i], "wom", "enter", "", NULL);
        console_handle_send(targets[i], "wom", "enter", NULL, "extra");
        console_handle_send(targets[i], "wom", "enter", "1", "extra");
        assert(requests == 6);
    }
}

static void test_calibration_commands(void)
{
    char *targets[] = {"0", "7", "all"};
    for (unsigned i = 0; i < 3; i++) {
        uint8_t target = i == 2 ? RCV_HID_TARGET_ALL : (i == 1 ? 7 : 0);
        requests = 0;
        console_handle_send(targets[i], "calibrate", NULL, NULL, NULL);
        assert(requests == 1 && last_target == target && last_flag == 0x02);
        console_handle_send(targets[i], "calibrate", "acc", NULL, NULL);
        assert(requests == 2 && last_target == target && last_flag == 0x03);
        console_handle_send(targets[i], "6-side", NULL, NULL, NULL);
        assert(requests == 3 && last_target == target && last_flag == 0x03);

        console_handle_send(targets[i], "calibrate", "gyro", NULL, NULL);
        console_handle_send(targets[i], "calibrate", "acc", "extra", NULL);
        console_handle_send(targets[i], "calibrate", "acc", "extra", "more");
        console_handle_send(targets[i], "6-side", "extra", NULL, NULL);
        assert(requests == 3);
    }
    puts("console: calibration aliases select existing wire operations; invalid args never queue");
}

static void test_u8_decimal(void)
{
    uint8_t value = 91;
    assert(parse_u8_arg("254", &value) && value == 254);
    assert(parse_u8_arg("+255", &value) && value == 255);
    assert(parse_u8_arg("-0", &value) && value == 0);
    char wrapped[4 * sizeof(unsigned long) + 4];
    snprintf(wrapped, sizeof(wrapped), "-%lu", ULONG_MAX);
    assert(parse_u8_arg(wrapped, &value) && value == 1);
    assert(!parse_u8_arg("256", &value) && value == 1);
    assert(!parse_u8_arg("", &value) && value == 1);
}

static void test_bounded_decimal(void)
{
    long value = -99;
    assert(parse_long_bounded("+100", 0, 100, &value) && value == 100);
    assert(parse_long_bounded("-0", 0, 255, &value) && value == 0);
    assert(parse_long_bounded("010", 0, 255, &value) && value == 10);
    assert(parse_long_bounded("86400", 0, 86400, &value) && value == 86400);
    const char *invalid[] = {NULL, "", " 1", "1 ", "1x", "0x10", "-1",
                             "86401", "999999999999999999999999999999999"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
        value = -99;
        assert(!parse_long_bounded(invalid[i], 0, 86400, &value));
        assert(value == -99);
    }
    assert(!parse_long_bounded("0", 1, 100, &value));
    assert(!parse_long_bounded("101", 1, 100, &value));
}

static void test_send_decimal(void)
{
    requests = 0;
    console_handle_send("+255", "shutdown", NULL, NULL, NULL);
    assert(requests == 1 && last_target == 255 && last_flag == ESB_PONG_FLAG_SHUTDOWN);
    console_handle_send("-0", "shutdown", NULL, NULL, NULL);
    assert(requests == 2 && last_target == 0);
    console_handle_send("010", "shutdown", NULL, NULL, NULL);
    assert(requests == 3 && last_target == 10);
    char *invalid_ids[] = {"", " 1", "1 ", "-1", "256", "0x10",
                           "999999999999999999999999999999999"};
    for (unsigned i = 0; i < sizeof(invalid_ids) / sizeof(invalid_ids[0]); i++) {
        console_handle_send(invalid_ids[i], "shutdown", NULL, NULL, NULL);
        assert(requests == 3);
    }

    char line[] = "send all channel +100 ignored ignored";
    char *argv[8];
    assert(parse_args(line, argv, 8) == 6);
    console_handle_send(argv[1], argv[2], argv[3], argv[4], argv[5]);
    assert(requests == 4 && last_value == 100);
    console_handle_send("all", "channel", "-0", NULL, NULL);
    assert(requests == 5 && last_value == 0);
    console_handle_send("all", "channel", "101", NULL, NULL);
    console_handle_send("7", "channel", "1", NULL, NULL);
    assert(requests == 5);

    console_handle_send("7", "sens", "auto", "y", NULL);
    assert(requests == 6 && last_target == 7 && last_axis == 1 && last_value == 0);
    console_handle_send("7", "sens", "auto", "y", "");
    assert(requests == 7 && last_value == 0);
    console_handle_send("7", "sens", "auto", "z", "+100");
    assert(requests == 8 && last_axis == 2 && last_value == 100);
    console_handle_send("7", "sens", "auto", "x", "0");
    console_handle_send("7", "sens", "auto", "x", "101");
    assert(requests == 8);

    console_handle_send("all", "test", "on", NULL, NULL);
    assert(requests == 9 && last_target == 255 && last_value == 0);
    console_handle_send("all", "test", "on", "", NULL);
    assert(requests == 10 && last_value == 0);
    console_handle_send("0", "test", "on", "+1000", NULL);
    assert(requests == 11 && last_target == 0 && last_value == 1000);
    console_handle_send("0", "test", "on", "-0", NULL);
    assert(requests == 12 && last_value == 0);
    console_handle_send("0", "test", "on", "-1", NULL);
    console_handle_send("0", "test", "on", "1001", NULL);
    assert(requests == 12);
}

int main(void)
{
    test_bounded_decimal();
    test_u8_decimal();
    test_send_decimal();
    test_calibration_commands();
    test_wom_commands();
    char *targets[] = {"0", "all"};
    for (unsigned i = 0; i < 2; i++) {
        requests = 0;
        console_handle_send(targets[i], "tcal", "heat", NULL, NULL);
        console_handle_send(targets[i], "tcal", "heat", "stop", NULL);
        console_handle_send(targets[i], "tcal", "heat", "status", NULL);
        console_handle_send(targets[i], "tcal", "heat", "44", NULL);
        console_handle_send(targets[i], "tcal", "heat", "start", "44");
        console_handle_send(targets[i], "tcal", "heat", "start", "nan");
        assert(requests == 0);
        console_handle_send(targets[i], "tcal", "heat", "start", NULL);
        assert(requests == 1 && last_target == (i ? RCV_HID_TARGET_ALL : 0));
        assert(last_flag == ESB_PONG_FLAG_TCAL_HEATED_START);
        console_handle_send(targets[i], "tcal", "auto", "on", NULL);
        assert(requests == 2 && last_flag == ESB_PONG_FLAG_TCAL_AUTO_ON);
        console_handle_send(targets[i], "tcal", "off", NULL, NULL);
        assert(requests == 3 && last_flag == ESB_PONG_FLAG_TCAL_OFF);
    }
    puts("console: heated start rejects unsupported controls/targets without queuing");
    return 0;
}
