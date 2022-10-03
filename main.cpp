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
 * GND = GND = 1
 * VCC = 3.3V = 2
 * MOSI = PC_12 = 6 
 * MISO = PC_11 = 7 
 * SCK = PC_10 = 5 
 * CSN = PC_9 = 4
 * CE = PC_8 = 3
 * IRQ = PC_7 = 8
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
#include <BufferedSerial.h>
#include <RS485.h>
#include <mbed.h>
#include <stdio.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>
#include <string.h>
#include "main.h"
#include "nRF24L01P.h"

UART_HandleTypeDef huart2; //Serial Monitor
UART_HandleTypeDef huart3; //Arduino Coms
SPI_HandleTypeDef hspi3;   //Tranceivers
RS485 RS485(); //Need UART pins Tx, Rx, GPIO
PwmOut motor1(PA_0);       //Motor PWM Pin: PA_0

Thread nRF24Thread;
Thread packetParserThread;
Thread motorControlThread;
Thread RS485Thread;

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

//RF Tx and Rx Variables
static uint8_t TRANSFER_SIZE = 32;
char rxData[32]; //"#000000134111154132000000"
int rxDataCnt = 0;
char txData[30];
bool newData;

//Buffers
char subtext3[4];   //3 Digit Buffer
char subtext1[2];   //1 Digit Buffer

int main()
{
    //Calling all init functions!
    HAL_Init();             
    SystemClock_Config();   
    Init_OnBoard_LEDs();
    GPIO_Init();
    USART2_UART_Init();
    USART3_UART_Init();
    SPI3_Init();

    //Starting of threads:
    nRF24Thread.start(nRF24);
    packetParserThread.start(packetParser);
    //motorControlThread.start(motorControl);
    RS485Thread.start(RS485TxRx);

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

void RS485TxRx()
{
    printf("Starting\n");
    ho = 1;                  // Enable sending on MAX485
    RS485.sendMsg(data, sizeof(data));
    HAL_Delay(600);           // Must wait for all the data to be sent   
    ho = 0;                  // Enable receiving on MAX485
    printf("Getting data\n");
    if(RS485.readable() >0)
    {
        memset(regvalue,0,sizeof(regvalue));
        HAL_Delay(200);
        RS485.recvMsg(regvalue,sizeof(data),500);
        HAL_Delay(200);
        for (int count = 0; count < 9; count++) 
        {
            printf("%X - ", regvalue[count]);
        }
    }
    else printf("No Data\n");
    printf("Done\n");
    HAL_Delay(1000);
    }
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
        memcpy(subtext3, &rxData[1], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &L1);
        L1 = map(L1, 0, 255, 27, 48);
        //L2
        memcpy(subtext3, &rxData[4], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &L2);
        L2 = map(L2, 0, 255, 27, 48);
        //LeftHatX
        memcpy(subtext3, &rxData[7], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &LeftHatX);
        LeftHatX = map(LeftHatX, 255, 0, 27, 48);
        //LeftHatY
        memcpy(subtext3, &rxData[10], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &LeftHatY);
        LeftHatY = map(LeftHatY, 255, 0, 27, 48);
        //RightHatX
        memcpy(subtext3, &rxData[13], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &RightHatX);
        RightHatX = map(RightHatX, 255, 0, 27, 48);
        //RightHatY
        memcpy(subtext3, &rxData[16], 3);
        subtext3[3] = '\0';
        sscanf(subtext3, "%d", &RightHatY);
        RightHatY = map(RightHatY, 255, 0, 27, 48);
        //R1
        memcpy(subtext1, &rxData[19], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &R1);
        //R2
        memcpy(subtext1, &rxData[20], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &R2);
        //Triangle
        memcpy(subtext1, &rxData[21], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &Triangle);
        //Circle
        memcpy(subtext1, &rxData[22], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &Circle);           
        //Cross
        memcpy(subtext1, &rxData[23], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &Cross);
        //Square
        memcpy(subtext1, &rxData[24], 1);
        subtext1[1] = '\0';
        sscanf(subtext1, "%d", &Square);     

        printf("L1: %d|L2: %d|LeftHatX: %d|LeftHatY: %d|RightHatX: %d|RightHatY: %d|R1: %d|R2: %d|Triangle: %d|Circle: %d|Cross: %d|Square: %d|\n", L1, L2, LeftHatX, LeftHatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square);        
        }
    }
}

void nRF24()
{
    /*****Transceiver Initialization*****/
    nRF24L01P nRF(PC_12, PC_11, PC_10, PC_9, PC_8, PC_7); //mosi, miso, sck, csn, ce, irq
    
    nRF.powerUp();
    nRF.setTransferSize(TRANSFER_SIZE);

    printf("nRF24L01+ Frequency    : %d MHz\r\n",  nRF.getRfFrequency());
    printf("nRF24L01+ Output power : %d dBm\r\n",  nRF.getRfOutputPower());
    printf("nRF24L01+ Data Rate    : %d kbps\r\n", nRF.getAirDataRate());
    printf("nRF24L01+ TX Address   : 0x%010llX\r\n", nRF.getTxAddress());
    printf("nRF24L01+ RX Address   : 0x%010llX\r\n", nRF.getRxAddress());
    printf("nRF24L01+ Transfer Size: %d\r\n", nRF.getTransferSize());
    
    nRF.setReceiveMode(); 
    nRF.enable(); 
    /***********************************/
    while(1)
    {
        //Receive data if there is data to be received
        if (nRF.readable())
        {
            rxDataCnt = nRF.read(NRF24L01P_PIPE_P0, rxData, sizeof( rxData ));
            newData = true;
        }
        if (newData == true)
        {
            printf("Received: %s\n", rxData);
            newData = false;
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
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    //Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    //Configure GPIO pins:
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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    //SPI3 GPIO Configuration
    //PC10     ------> SPI3_SCK
    //PC11     ------> SPI3_MISO
    //PC12     ------> SPI3_MOSI
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Init_OnBoard_LEDs(void)
{
    //Initialize main LEDs 
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef BoardLEDs;
	BoardLEDs.Mode = GPIO_MODE_OUTPUT_PP;
	BoardLEDs.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD, &BoardLEDs);
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

static void SPI3_Init(void)
{
  //SPI3 parameter configuration
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
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
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}
