/* Walrus ROV
 * STM32F4 ROV 
 * /09/2022
 * Alexander, Tony, Clinton
 *
 * PS3 Controls Packet Format: 
 *  1  4   7  10   13  16  19 20 21 22 23 24 25 28
 * "# 000 000 000 000 000 000 0  0  0  0  0  0  000" = 28
 * Pound, L1, L2, LeftHatX, LefthatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square, Checksum
 * 
 * //Motor Pin Initializations:
 * PwmOut motor1(PB_7);       //Motor: Front Z
 * PwmOut motor2(PB_6);       //Motor: Rear Z
 * PwmOut motor3(PE_6);       //Motor: Front Right
 * PwmOut motor4(PF_8);       //Motor: Front Left
 * PwmOut motor5(PF_7);       //Motor: Back Right
 * PwmOut motor6(PF_6);       //Motor: Back Left
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
 */
#include <mbed.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "cmsis.h"

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

UART_HandleTypeDef huart2; //Serial Monitor

Thread packetParserThread; //Thread to parse packets into below variables
Thread motorControlThread; //Thread that takes parsed packet data and maps to DC for motors 
Thread nRF24Thread;        //Thread that handles data reception from transceivers 

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
uint8_t RS485_RxDataBuf[32] = {0};
uint8_t RS485_RxData[32] = {0};                              //"#99RANDOMSENSORDATABABYOHYA999"
uint8_t RS485_TxData[32] = "#99RANDOMSENSORDATABABYOHYA999"; //"#99000000134111154132000000999"
size_t old_posRx;
size_t posRx;
bool newData = false;

//Packet Parsing Buffers 
char subtext3[4];   //3 Digit Buffer
char subtext1[2];   //1 Digit Buffer

int main()
{
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    
    SystemClock_Config();       //Initialize Clocks 
    GPIO_Init();                //Initialize all GPIO Pins for Periph's and Outputs
    USART2_UART_Init();         //Initialize UART2 for the Serial Monitor
    USART3_UART_Init();         //Initialize UART3 for RS485 Coms
    
    motorControlThread.start(motorControl);     //Start Motor Control (Init's as OFF)
    packetParserThread.start(packetParser);     //Start Parsing of Received Wireless Packets 
    nRF24Thread.start(nRF24);                   //Start Rx and Tx for Wireless Packets
    
    DMA_Start_Transmit();               //Start DMA Transmission on UART3 for RS485

    while(1)
    {
        LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_12);
        HAL_Delay(1000);
        
        printf("Raw Data: %s\n", RS485_RxDataBuf);
        printf("Filtered Data: %s\n", RS485_RxData);
        printf("L1: %d|L2: %d|LeftHatX: %d|LeftHatY: %d|RightHatX: %d|RightHatY: %d|R1: %d|R2: %d|Triangle: %d|Circle: %d|Cross: %d|Square: %d|\n", L1, L2, LeftHatX, LeftHatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square);
        
       //Polling USART Transmit with DMA Debugging:
        /*for (int i = 0; i < 33; i++)
        {
            HAL_Delay(1);
            LL_USART_TransmitData8( USART1, RS485_TxData[i]);
        }*/
        //printf("DMA Buffer: %s\n", RS485_RxDataBuf); //DMA Buffer
        //uint8_t length = LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_3);
        //printf("Tx Length: %d\n", length);
        //uint8_t len2 = LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
        //printf("Rx Length: %d\n", len2);
    }
}

/***ACTIVE FUNCTIONS***/
void DMA_Start_Transmit(void)
{
    //Disable the channel or stream to be reconfigured before Tx
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
    //Configure DMA memory addresses for UART to grab from
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)RS485_TxData);
    //Configure DMA length (number of bytes to transmit)
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, ARRAY_LEN(RS485_TxData));
    //Enabe UART DMA Tx Request
    LL_USART_EnableDMAReq_TX(USART3); //this line freeze
    //Enable UART Tx Direction
    LL_USART_EnableDirectionTx(USART3); //Not sure if needed 
    //Clear all flags
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_HT3(DMA1);
    LL_DMA_ClearFlag_DME3(DMA1);
    LL_DMA_ClearFlag_FE3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    //Enable DMA stream or channel to start tranmission
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3); 
}

void USART_Rx_Check(void) 
{
    //Function called on every TC interrupt or IDLE line interrupt
    posRx = ARRAY_LEN(RS485_RxDataBuf) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
    
    if (posRx > old_posRx) 
    {                 
        //Check if the new position in the buffer is greater than the old (Bad timed packet)
        USART_Process_Data(&RS485_RxDataBuf[old_posRx], posRx - old_posRx);
        old_posRx = posRx;
    } 
    else if (posRx < old_posRx)
    {
        //Check if the new position in the buffer is less than the old (Bad packet)
        USART_Process_Data(&RS485_RxDataBuf[old_posRx], ARRAY_LEN(RS485_RxDataBuf) - old_posRx);
        old_posRx = posRx;                       
    }
    else if (posRx == 0)
    {
        //Check if the new position in the buffer is the same as the old (Good packet)
        USART_Process_Data(&RS485_RxDataBuf[0], 32); 
        old_posRx = posRx;
    }
}

void USART_Process_Data(uint8_t* data, size_t len) 
{
    memcpy(RS485_RxData, &data, 32);
    newData = true;
}

void motorControl()
{
    //Thread that control PWM signal outputs within parameters below, uses global parsed data
    //Initialize PWM: 3.9ms Period | 256Hz | High DC: 48% | Stop DC: 38% | Low DC: 28%
    uint8_t maxPowa = 41;
    uint8_t minPowa = 35;
    uint8_t stopPowa = 38;
    
    //Pin Initializations:
    PwmOut motor1(PB_7);       //Motor: Front Z
    PwmOut motor2(PB_6);       //Motor: Rear Z
    PwmOut motor3(PE_6);       //Motor: Front Right
    PwmOut motor4(PF_8);       //Motor: Front Left
    PwmOut motor5(PF_7);       //Motor: Back Right
    PwmOut motor6(PF_6);       //Motor: Back Left

    //Period / Frequency Initializations:
    motor1.period(0.0039);
    motor2.period(0.0039);
    motor3.period(0.0039);
    motor4.period(0.0039);
    motor5.period(0.0039);
    motor6.period(0.0039);

    float motor1DC; 
    float motor2DC;
    float motor3DC;
    float motor4DC;
    float motor5DC;
    float motor6DC;

    while (1)
    {
        //Motor control loop; deadzone programmed with < > values; only ever 2 motors on at once
        //If none of these conditions are met, returns all motors to off PWM of 38%
        if (LeftHatY > 115 && LeftHatY < 255)
        {
            //Move Forward (motor 5 and motor 6: 48% > DC > 38%)
            motor5DC = (float)(map(LeftHatY, 255, 0, minPowa, maxPowa))/100.0;
            motor6DC = (float)(map(LeftHatY, 255, 0, minPowa, maxPowa))/100.0;

            motor5.write(motor5DC);
            motor6.write(motor6DC);
        }
        else if (LeftHatY < 110 && LeftHatY > 0)
        {
            //Move Backwards: (motor 5 and motor 6: 38% > DC > 28%)
            motor5DC = (float)(map(LeftHatY, 255, 0, minPowa, maxPowa))/100.0;
            motor6DC = (float)(map(LeftHatY, 255, 0, minPowa, maxPowa))/100.0;

            motor5.write(motor5DC);
            motor6.write(motor6DC);
        }
        else if (LeftHatX < 130 && LeftHatX > 0)
        {
            //Move Left: (motor 5 and motor 3: 48% > DC > 38%)
            motor5DC = (float)(map(LeftHatX, 255, 0, minPowa, maxPowa))/100.0;
            motor3DC = (float)(map(LeftHatX, 255, 0, minPowa, maxPowa))/100.0;

            motor5.write(motor5DC);
            motor3.write(motor3DC);
        }
        else if (LeftHatX > 140 && LeftHatX < 255)
        {
            //Move Right: (motor 6 and motor 4: 48% > DC > 38%)
            motor6DC = (float)(map(LeftHatX, 255, 0, minPowa, maxPowa))/100.0;
            motor4DC = (float)(map(LeftHatX, 255, 0, minPowa, maxPowa))/100.0;

            motor6.write(motor6DC);
            motor4.write(motor4DC);

        }
        else if (L1 > 10 && L1 < 255)
        {
            //Move Up (motor 1 and motor 2: 38% > DC > 48%)
            motor1DC = (float)(map(L1, 0, 255, stopPowa, maxPowa))/100.0;
            motor2DC = (float)(map(L1, 0, 255, stopPowa, maxPowa))/100.0;

            motor1.write(motor1DC);
            motor2.write(motor2DC);
        }
        else if (L2 > 10 && L2 < 255)
        {
            //Move Down (motor 1 and motor 2: 28% > DC > 38%)
            motor1DC = (float)(map(L2, 0, 255, stopPowa, minPowa))/100.0;
            motor2DC = (float)(map(L2, 0, 255, stopPowa, minPowa))/100.0;

            motor1.write(motor1DC);
            motor2.write(motor2DC);
        }
        else
        {
            //Idle mode
            motor1.write(stopPowa);
            motor2.write(stopPowa);
            motor3.write(stopPowa);
            motor4.write(stopPowa);
            motor5.write(stopPowa);
            motor6.write(stopPowa);
        }
    }
}

void packetParser()
{
    while(1)
    {
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

            newData = false; //Reset new data state
        }
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/***INITIALIZATIONS***/
static void GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    //GPIO Ports Clock Enable
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);

    //Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    //Configure GPIO for on board LED's
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //Configure GPIO pins for UART3
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //Setup PWM Pins NOT SURE IF NEEDED
   /* __HAL_RCC_TIM2_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    // Initializes the RCC Oscillators according to the specified parameters
    // in the RCC_OscInitTypeDef structure.
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
    // Initializes the CPU, AHB and APB buses clocks
    
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
    //Initialize USART2 for Serial Monitor (Using HAL, will switch to LL)
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
    //Initialize USART3 for DMA Rx and Tx
    LL_USART_InitTypeDef USART_InitStruct = {0};
    
    //Peripheral clock enable
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    //USART3 DMA Init for Rx
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
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)RS485_RxDataBuf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ARRAY_LEN(RS485_RxDataBuf));

    //DMA interrupt init for Rx
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    //USART3 DMA Init for Tx (Some setup performed in transmit function)
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

    //DMA interrupt init for Tx
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);
    NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    //USART Config (Tx done in DMA transmit function)
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
    LL_USART_EnableIT_IDLE(USART3);

    //USART interrupt init 
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);

    //Enable USART3 and DMA Rx Stream 
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    LL_USART_Enable(USART3);
}

/***INTERRUPT HANDLERS***/
void DMA1_Stream1_IRQHandler(void) 
{
    // Check transfer-complete interrupt for Rx
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) 
    {
        LL_DMA_ClearFlag_TC1(DMA1);             //Clear transfer complete flag 
        USART_Rx_Check();                       //Filter received data
    }
}

void DMA1_Stream3_IRQHandler(void) 
{
    //Check transfer-complete interrupt for Tx
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_3) && LL_DMA_IsActiveFlag_TC3(DMA1)) {
        LL_DMA_ClearFlag_TC3(DMA1);             //Clear transfer complete flag
        DMA_Start_Transmit();           //Send more data
    }
}

void USART3_IRQHandler(void) 
{
    //Check idle line interrupt for USART
    if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) {
        LL_USART_ClearFlag_IDLE(USART3);        //Clear IDLE line Flag
        USART_Rx_Check();                       //Filter received data
    }
}

void Error_Handler(void)
{
    //User can add his own implementation to report the HAL error return state
    __disable_irq();
    while (1)
    {
    }
}