#ifndef __BSP_USARTX_H__
#define __BSP_USARTX_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define USARTx                                 USART1
#define USARTx_BAUDRATE                        115200
#define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART1_CLK_ENABLE()
#define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART1_CLK_DISABLE()

#define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define USARTx_Tx_GPIO_PIN                     GPIO_PIN_6
#define USARTx_Tx_GPIO                         GPIOB
#define USARTx_Rx_GPIO_PIN                     GPIO_PIN_7   
#define USARTx_Rx_GPIO                         GPIOB

#define USARTx_AFx                             GPIO_AF7_USART1

#define USARTx_IRQHANDLER                      USART1_IRQHandler
#define USARTx_IRQn                            USART1_IRQn


#define USART2x                                 USART2
#define USART2x_BAUDRATE                        115200
#define USART2_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define USART2_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()
#define USART2x_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define USART2x_Tx_GPIO_PIN                     GPIO_PIN_5
#define USART2x_Tx_GPIO                         GPIOD
#define USART2x_Rx_GPIO_PIN                     GPIO_PIN_6   
#define USART2x_Rx_GPIO                         GPIOD
#define USART2x_AFx                             GPIO_AF7_USART2
#define USART2x_IRQHANDLER                      USART2_IRQHandler
#define USART2x_IRQn                            USART2_IRQn


/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx;
extern UART_HandleTypeDef husart2;

/* 函数声明 ------------------------------------------------------------------*/
void MX_USARTx_Init(void);
void vcan_sendware(uint8_t *wareaddr, uint32_t waresize);


#endif  /* __BSP_USARTX_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
