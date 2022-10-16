#ifndef __MAIN_H

#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"

#ifndef MAIN_H
#define MAIN_H

//Initializations & Configs (Stuff Called Once)
void SystemClock_Config(void);
static void SPI3_Init(void);
static void USART2_UART_Init(void);
static void GPIO_Init(void);
static void USART3_UART_Init(void);

//Active Functions
long map(long x, long in_min, long in_max, long out_min, long out_max);
void nRF24(); //Operates in its own thread
void packetParser(); //Operates in its own thread
void Error_Handler(void);
void motorControl();
void DMA1_Stream1_IRQHandler(void);
void USART3_IRQHandler(void);
void USART_Rx_Check(void);
void USART_Process_Data(uint8_t* data, size_t len);
void USART3_DMA2_Start_Transmit(void);

#endif

#ifdef __cplusplus

}
#endif

#endif /* __MAIN_H */