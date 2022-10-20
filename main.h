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

//Initializations & Configs (Stuff Called Once)
void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART2_UART_Init(void);
static void USART3_UART_Init(void);

//Active Functions
long map(long x, long in_min, long in_max, long out_min, long out_max);
void packetParser(); //Operates in its own thread
void Error_Handler(void);
void motorControl();
void USART_Rx_Check(void);
void USART_Process_Data(uint8_t* data, size_t len);
void DMA_Start_Transmit(void);

//Interrupt Handlers
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void USART3_IRQHandler(void);

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

#ifdef __cplusplus

}
#endif

#endif /* __MAIN_H */