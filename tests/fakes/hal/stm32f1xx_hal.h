#ifndef __STM32F1xx_HAL_FAKE_H__
#define __STM32F1xx_HAL_FAKE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#ifndef __IO
#define __IO volatile
#endif

/**
  * @brief  HAL Handle for UART
  */
typedef struct _UART_HandleTypeDef {
    int handle;
} UART_HandleTypeDef;

/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

/**
  * @brief TIM Fake Timers
  * copied values from real description of hw
  */
typedef struct
{
  __IO uint32_t CCR1; /*!< TIM capture/compare register 1,              Address offset: 0x34 */
  __IO uint32_t CCR2; /*!< TIM capture/compare register 2,              Address offset: 0x38 */
  __IO uint32_t CCR3; /*!< TIM capture/compare register 3,              Address offset: 0x3C */
  __IO uint32_t CCR4; /*!< TIM capture/compare register 4,              Address offset: 0x40 */

} TIM_TypeDef;

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_FAKE_H__ */