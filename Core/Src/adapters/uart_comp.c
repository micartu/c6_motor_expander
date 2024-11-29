#include "uart_comp.h"

HAL_StatusTypeDef WRP_UART_Receive_IT(UART_HandleTypeDef * huart, uint8_t * pData, uint16_t size)
{
    return HAL_UART_Receive_IT(huart, pData, size);
}