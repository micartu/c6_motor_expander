#ifndef __UART_WRP_H__
#define __UART_WRP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/** 
 * @method WRP_UART_Receive_IT
 * @abstract a wrapper around HAL_UART_Receive_IT which receives data from UART
 * 
 * @param huart a handle for uart manipulation
 * @param data a buffer where to put received bytes from UART
 * @param size buffer's maximum size
 * 
 * @result an enum which describes successfulnes of the operation
*/
HAL_StatusTypeDef WRP_UART_Receive_IT(UART_HandleTypeDef * huart, uint8_t * data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __UART_WRP_H__ */