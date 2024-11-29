#include "uart_comp.h"
#include <string.h>

static uint8_t * _last_call_buf = NULL;
static size_t _last_call_read_len = 0;
static UART_HandleTypeDef * _last_huart = NULL;
static const uint8_t * _data = NULL;
static size_t _size;
static size_t _read;
static size_t _called = 0;
static uint16_t _last_size;
static HAL_StatusTypeDef _ret = HAL_OK;

extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);

#define MIN(a,b) (((a)<(b)) ? (a) : (b))

static void _read_data(uint8_t * data, const size_t size)
{
    if (_read < _size)
    {
        const size_t rsz = MIN(size, _size - _read);
        memcpy(data, _data + _read, rsz);
        _read += rsz;
        _last_size = rsz;
    }
}

void _install_data_for_UART_receive(const uint8_t * data, uint16_t size)
{
    _data = data;
    _size = size;
    _read = 0;
    _last_size = 0;
    // do we have a pending request for read data?
    if (_last_call_buf != NULL)
    {
        _read_data(_last_call_buf, _last_call_read_len);
        _last_call_buf = NULL;
        _last_call_read_len = 0;
        HAL_UART_RxCpltCallback(_last_huart);
        _last_huart = NULL;
    }
}

void _deinstall_data_for_UART_receive()
{
    _size = 0;
    _read= 0;
    _data = NULL;
    _last_huart = NULL;
    _last_call_buf = NULL;
    _last_call_read_len = 0;
    _last_size = 0;
}

void _install_ret_type_for_UART_receive(HAL_StatusTypeDef ret)
{
    _ret = ret;
}

uint16_t _return_last_read_size()
{
    return _last_size;
}

HAL_StatusTypeDef WRP_UART_Receive_IT(UART_HandleTypeDef * huart, uint8_t * data, uint16_t size)
{
    if (_data != NULL)
        _read_data(data, size);
    else { // there is no data to be read yet
        _last_huart = huart;
        _last_call_buf = data;
        _last_call_read_len = size;
        return HAL_BUSY;
    }
    return _ret;
}

#undef MIN