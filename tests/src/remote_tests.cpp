#include "CppUTest/TestHarness.h"
#include "remote.h"
#include <string.h>

#define NOT_USED 0xAA
#define BUF_SZ 32
#define SAMPLE_DATA PACK_BEGIN, 7, CMD_TYPE_CUST, 0x20, 0, 0, 0, 0x29, PACK_END
#define SAMPLE_PACKET { SAMPLE_DATA }

static uint32_t _kgain = 0;
static uint8_t _kcmd = 0;
static size_t _kgain_called = 0;

// needed headers + and implementations which control fake's behavior:
extern "C"
{
    void _install_data_for_UART_receive(const uint8_t * data, uint16_t size);
    void _install_ret_type_for_UART_receive(HAL_StatusTypeDef ret);
    void _deinstall_data_for_UART_receive();
    uint16_t _return_last_read_size();
}

static void custom_data_callback(uint32_t k)
{
    _kgain = k;
    ++_kgain_called;
}

static void usb_command_received(uint8_t cmd, const void *buf, size_t sz)
{
    _kcmd = cmd;
}

TEST_GROUP(RemoteTestGroup)
{
    struct UART_Descr com;
    uint8_t buf[BUF_SZ];

    void setup()
    {
        memset(buf, NOT_USED, sizeof(buf));
        init_communication_uart(
            &com,
#ifdef USING_REAL_UART
            &huart,
#endif
            buf,
            sizeof(buf));
        _kgain = 0;
        _kgain_called = 0;
        com.void_cmd_p_ptr = &usb_command_received;
        com.custom_cmd_p_ptr = &custom_data_callback;
    }

    void teardown()
    {
        deinit_communication_uart(&com);
#ifdef USING_REAL_UART
        _deinstall_data_for_UART_receive();
#endif
    }
};

TEST(RemoteTestGroup, CustomDataHandler)
{
    // given
    uint8_t data[] = { 0x51, 0x9, 0xb, 0xff, 0xff, 0x0, 0xff, 0xff, 0x0, 0xb, 0xab };

    // when
    uart_over_usb_data_received_handler(&com, data, sizeof(data));

    // then
    LONGS_EQUAL(CMD_MOTOR_PWM, _kcmd);
}

TEST(RemoteTestGroup, ForBytesDataHandler)
{
    // given
    uint8_t data[] = SAMPLE_PACKET;

    // when
    uart_over_usb_data_received_handler(&com, data, sizeof(data));

    // then
    LONGS_EQUAL(1, _kgain_called);
}