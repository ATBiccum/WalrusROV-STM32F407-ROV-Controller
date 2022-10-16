/* Walrus ROV
 * STM32F4 ROV 
 * 28/09/2022
 * Alexander, Tony, Clinton
 *
 * 
 * TODO:
 *  1. Apply parsed packets to motor control maps
 *  2. When no packet received: force motors to off
 *
 * 
 *
 * DONE:
 * 1. Consistent wireless coms 
 * 2. Test packet parsing and verification
 *
 * CONFIGS:
 *
 * PS3 Controls Packet Format: 
 *  1  4   7  10   13  16  19 20 21 22 23 24 25 28
 * "# 000 000 000 000 000 000 0  0  0  0  0  0  000" = 28
 * Pound, L1, L2, LeftHatX, LefthatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square, Checksum
 * 
 *
 * RESORUCES:
 * https://os.mbed.com/users/Owen/code/nRF24L01P/
 * https://os.mbed.com/cookbook/nRF24L01-wireless-transceiver
 * https://forums.mbed.com/t/hitchhikers-guide-to-printf-in-mbed-6/12492
 * https://controllerstech.com/how-to-setup-uart-using-registers-in-stm32/
 * https://arduino.ua/docs/AfroESC30A.pdf
 * https://os.mbed.com/handbook/PwmOut
 * https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 * https://os.mbed.com/docs/mbed-os/v6.15/apis/thread.html
 * https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123
 * https://os.mbed.com/users/belloula/code/mbed_blinko//file/84c281baac9c/main.cpp/
 *
 *
 */
#include <mbed.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "cmsis.h"

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

UART_HandleTypeDef huart2; //Serial Monitor
UART_HandleTypeDef huart3; //RS485 Coms
PwmOut motor1(PA_0);       //Motor PWM Pin: PA_0

Thread packetParserThread;
Thread motorControlThread;

//Packet from Control Station Parsed Variables
int L1;
int L2;
int LeftHatX;
int LeftHatY;
int RightHatX;
int RightHatY;
int R1;
int R2;
int Triangle; 
int Circle;
int Cross;
int Square;

//RS485 Tx and Rx Variables
uint8_t RS485_RxData[32] = {0};
uint8_t RS485_TxData[32] = "#99900000013411115413200000099";
size_t old_posRx;
size_t posRx;
bool newData = false;

//Buffers
char subtext3[4];   //3 Digit Buffer
char subtext1[2];   //1 Digit Buffer

int main()
{
    //Calling all init functions!
    HAL_Init();             
    SystemClock_Config();   
    GPIO_Init();
    USART2_UART_Init();
    //DMA_Init();
    USART3_UART_Init();
    
    //Starting of threads:
    //nRF24Thread.start(nRF24);
    //packetParserThread.start(packetParser);
    //motorControlThread.start(motorControl);
    while(1)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        HAL_Delay(500);
        USART3_DMA2_Start_Transmit();
        printf("Received Data: %s", RS485_RxData);
    }
}
void USART3_DMA2_Start_Transmit(void)
{
    /*
     * Steps to Transmit on USART using DMA: In Order
     * Configure DMA memory & peripheral addresses for UART and your buffer
     * Configure DMA length (number of bytes to transmit in your case)
     * Enable UART DMA TX request (not done in your example)
     * Enable UART TX mode
     * Enable UART itself
     * Enable DMA channel
     * Transmission will start
     */

    //Disable the channel to be reconfigured before Tx
    LL_USART_Disable(USART3);
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
    //Configure DMA memory & peripheral addresses for UART and your buffer
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, LL_USART_DMA_GetRegAddr(USART3));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)RS485_TxData);
    //Configure DMA length 
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, ARRAY_LEN(RS485_TxData));
    //Enabe DMA Tx Request
    LL_USART_EnableDMAReq_TX(USART3);
    //Clear all flags
    LL_DMA_ClearFlag_TC4(DMA1);
    //Enable UART Tx Mode
    LL_USART_Enable(USART3);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3); //Enable DMA Channel for Tx
}

void USART_Rx_Check(void) 
{
    //Function called on every TC interrupt
    posRx = ARRAY_LEN(RS485_RxData) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
    //See github tutorial for explanation
    if (posRx > old_posRx) 
    {                 
        USART_Process_Data(&RS485_RxData[old_posRx], posRx - old_posRx);
        old_posRx = posRx;
    } 
    else if (posRx < old_posRx)
    {
        USART_Process_Data(&RS485_RxData[old_posRx], ARRAY_LEN(RS485_RxData) - old_posRx);
        old_posRx = posRx;                       
    }
    else if (posRx == 0)
    {
        //This will be the typical case
        USART_Process_Data(&RS485_RxData[0], 32);
        old_posRx = posRx;
    }
}

void USART_Process_Data(uint8_t* data, size_t len) 
{
    newData = true;
}

//Active Functions: (Called Often)
void motorControl()
{
    //Initialize PWM: 3.9ms Period | 256Hz | High DC: 48% | Stop DC: 38% | Low DC: 28%
    //motor1.period(0.0039);

    //int control;
    //int control1;
    //control1 = map(control, 255, 0, 27, 48);
    //float control2 = (float)control1 / 100.0;
    //motor1.write(control2);
    //printf("Received Controls |L1: %d|R1: %d|L2: %d|R2: %d|LeftHatX: %d|LeftHatY: %d|RightHatX: %d|RightHatY: %d|Triangle: %d|Circle: %d|Cross: %d|Square: %d|\n", L1, R1, L2, R2, LeftHatX, LeftHatY, RightHatX, RightHatY, Triangle, Circle, Cross, Square);
}

void packetParser()
{
    //Parser steps: 
    //1. If first value of packet is # 
    //2. Parse data into char variable
    //3. Convert char value to int and store in its proper variable
    //4. Map that value to corresponding duty cycle
    while(1)
    {
        //L1
        if (newData == true)
        {
        memcpy(subtext3, &RS485_RxData[1], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &L1);
        L1 = map(L1, 0, 255, 27, 48);
        //L2
        memcpy(subtext3, &RS485_RxData[4], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &L2);
        L2 = map(L2, 0, 255, 27, 48);
        //LeftHatX
        memcpy(subtext3, &RS485_RxData[7], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &LeftHatX);
        LeftHatX = map(LeftHatX, 255, 0, 27, 48);
        //LeftHatY
        memcpy(subtext3, &RS485_RxData[10], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &LeftHatY);
        LeftHatY = map(LeftHatY, 255, 0, 27, 48);
        //RightHatX
        memcpy(subtext3, &RS485_RxData[13], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &RightHatX);
        RightHatX = map(RightHatX, 255, 0, 27, 48);
        //RightHatY
        memcpy(subtext3, &RS485_RxData[16], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &RightHatY);
        RightHatY = map(RightHatY, 255, 0, 27, 48);
        //R1
        memcpy(subtext1, &RS485_RxData[19], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &R1);
        //R2
        memcpy(subtext1, &RS485_RxData[20], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &R2);
        //Triangle
        memcpy(subtext1, &RS485_RxData[21], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &Triangle);
        //Circle
        memcpy(subtext1, &RS485_RxData[22], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &Circle);           
        //Cross
        memcpy(subtext1, &RS485_RxData[23], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &Cross);
        //Square
        memcpy(subtext1, &RS485_RxData[24], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &Square);     

        printf("L1: %d|L2: %d|LeftHatX: %d|LeftHatY: %d|RightHatX: %d|RightHatY: %d|R1: %d|R2: %d|Triangle: %d|Circle: %d|Cross: %d|Square: %d|\n", L1, L2, LeftHatX, LeftHatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square);        
        }
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Error_Handler(void)
{
    //User can add his own implementation to report the HAL error return state
    printf("ERROR HANDLER TRIGGERED.\n");
    __disable_irq();
    while (1)
    {
    }
}

//Initializations and Config Functions: (Called Once)
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    //Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    //Configure GPIO for on board LED's
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //Configure GPIO pins for UART3
    __HAL_RCC_USART3_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //Setup PA_0 for TIM2 PWM
    /*__HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
    Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
    Error_Handler();
    }
}

static void USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void USART3_UART_Init(void)
{
        LL_USART_InitTypeDef USART_InitStruct = {0};

    //Peripheral clock enable
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    //USART3 DMA init for Stream 1 Channel 4 = Rx
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_1, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART3));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)RS485_RxData);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ARRAY_LEN(RS485_RxData));

    /* Enable HT & TC interrupts */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_3, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, LL_USART_DMA_GetRegAddr(USART3));

    /* Enable TC interrupt */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    /* USART configuration */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_EnableDMAReq_RX(USART3);
    //LL_USART_EnableDMAReq_TX(USART3);
    LL_USART_EnableIT_IDLE(USART3);

    /* USART interrupt */
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);

    /* Enable USART and DMA */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    LL_USART_Enable(USART3);
}

void DMA1_Stream1_IRQHandler(void) 
{
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);             /* Clear transfer complete flag */
        USART_Rx_Check();                       /* Check for data to process */
    }
}

void DMA1_Stream3_IRQHandler(void) 
{
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_3) && LL_DMA_IsActiveFlag_TC3(DMA1)) {
        LL_DMA_ClearFlag_TC3(DMA1);             /* Clear transfer complete flag */
        USART3_DMA2_Start_Transmit();                      /* Check for data to process */
    }
}

void USART3_IRQHandler(void) 
{
    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) 
    {
        LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
        USART_Rx_Check();                      /* Check for data to process */
    }
}