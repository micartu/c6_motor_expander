#ifndef __REMOTE_H__
#define __REMOTE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdio.h>

// #define USING_REAL_UART

// commands
#define CMD_TYPE_SET_K		0x7
#define CMD_TYPE_SET_P		0x8
#define CMD_TYPE_CUST		0x9
#define CMD_TYPE_SP		    0xa
#define CMD_MOTOR_PWM	    0xb

// sub-commands
#define CUSTOM_PING         10
#define CUSTOM_VEL_PID		11
#define CUSTOM_VEL_POS		14
#define CUSTOM_VEL		    15
#define CUSTOM_POS		    16
#define WHEEL_TICKS		    17

// packet structure constants

#define PACK_BEGIN			0x51
#define PACK_END			0xAB
#define PACKET_MIN_SZ		14
#define MAX_CACHED_SZ		2
#define SYS_TIMEOUT_TICKS	12

// descriptions

/// describes what part of packet we've received
enum UART_recv_state
{
    IDLE,           //!< no part of packet was received and not awaited
    WFIRST_BYTE,    //!< we're waiting for the first byte of the packet
    WLEN_BYTE,      //!< waiting for 2nd byte - length of the rest of the packet
    WREST_BYTES,    //!< waiting for rest of the packet determined at the prev. step
};

typedef void (*func_uint32_paramed_t)(uint32_t);
typedef void (*func_void_param_t)(uint8_t, const void*, size_t);

struct UART_Descr
{
#ifdef USING_REAL_UART
    UART_HandleTypeDef *huart;
#endif
    uint8_t * uart_buffer;
    size_t uart_ptr;
    size_t sz;
    size_t pack_len;
    size_t not_parsed_sz;
    uint32_t rticks_elapsed;
    func_uint32_paramed_t custom_cmd_p_ptr; //<! a callback to a custom function which could receive 32-bit data
    func_void_param_t void_cmd_p_ptr; //<! a callback to a custom function which could receive 32-bit data
    enum UART_recv_state rstate;
};

// avail. procedures

/**
 * Initializes the structure needed to perform communication over UART interface
 * 
 * @param com describes UART we're working with
 * @param huart a handle for UART we're working with
 * @param buffer blob of data where we'll store the packets' data received over UART
 * @param size of buffer
 */
void init_communication_uart(struct UART_Descr *com,
#ifdef USING_REAL_UART
                             UART_HandleTypeDef *huart,
#endif
                             uint8_t *buffer, size_t size);

/**
 * Installs a callback for custom function which could receive a uint32 parameter
 * 
 * @param com describes UART we're working with
 * @param func a pointer to a callback to be called
 */
void install_custom_callback(struct UART_Descr * com, func_uint32_paramed_t func);

/**
 * Function for being called from within the receiving IRQ handler
 * 
 * @param com describes UART we're working with
 */
// void uart_irq_handler(const UART_HandleTypeDef * huart);

/**
 * Function for being called from within the clock IRQ handler
 */
void uart_systick_handler();

/**
 * Function for normal operation of UART.
 * Proceeds with the data received in different IRQs.
 * Must be called from global run-loop on its every cycle.
 */
void uart_runloop_handler();

/**
 * Deinitializes and frees data in the structure needed to perform communication over UART interface.
 * Must be called after init_communication_uart, if we need to free the data.
 * Not always needed on real firmware though, but needed for tests
 * 
 * @param com describes UART we're working with
 */
void deinit_communication_uart(const struct UART_Descr * com);

#ifndef USING_REAL_UART
/**
 * parses the buffer with commands received elsewhere like e.g. at the USB-uart
 */
void uart_over_usb_data_received_handler(struct UART_Descr *com, uint8_t *buf, uint32_t len);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_H__ */