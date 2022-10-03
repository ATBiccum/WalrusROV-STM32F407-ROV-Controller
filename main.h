#include <mbed.h>
#include <stdio.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>
#include "nRF24L01P.h"

#ifndef MAIN_H
#define MAIN_H

//Initializations & Configs (Stuff Called Once)
void SystemClock_Config(void);
static void SPI3_Init(void);
static void USART2_UART_Init(void);
static void GPIO_Init(void);
void Init_OnBoard_LEDs(void);
static void USART3_UART_Init(void);
static void TIM2_Init(void);

//Active Functions
long map(long x, long in_min, long in_max, long out_min, long out_max);
void nRF24(); //Operates in its own thread
void packetParser(); //Operates in its own thread
void Error_Handler(void);
void motorControl();
void RS485TxRx();

#endif