/* Walrus ROV
 * STM32F4 ROV Controller
 * 
 * Objective: Send 6 x smoothed PWM signals to ESC's with values dependent on received controls over 
 * UART (RS422 chips -> UART) using DMA for UART receive. Has connection lost built into DMA 
 * buffers filter. Additionally sends sensor data from an IMU module and pressure sensor module
 * over UART to RS422 chips. 
 * 
 */

#include <mbed.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include "main.h" 
#include "cmsis.h"
#include <watchdog_api.h>

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

UART_HandleTypeDef huart2; //Serial Monitor
I2C_HandleTypeDef hi2c1;   //Pressure sensor and IMU

//Timers for PWM Pins
TIM_HandleTypeDef htim4;   
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

Thread packetParserThread; //Thread to parse packets
Thread motorControlThread; //Thread that takes parsed packet data and maps to duty cycle for motors 

//Parsed Variables for Controls
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

//RS422 Tx and Rx Variables
uint8_t RS422_RxDataBuf[500] = {0};
uint8_t RS422_RxData[500] = {0}; 
uint8_t RS422_TxData[32] = "#9900000000000000000000000999#";
uint8_t parseBuffer[32] = {0};
char subtext2[2] = {0};

//Packet Count Variables
int newPacketCount = 0;
int oldPacketCount = 0;
int transmitPacketCount = 0;

//Flags
bool newData = false;
bool motorsToggle = false; 
bool startFilterFlag = false;

//Packet Parsing Buffers 
char subtext3[4] = {0};   //3 Digit Buffer
char subtext1[2] = {0};   //1 Digit Buffer

//IMU Variables
uint8_t eulXLSB = 0x1A;
uint8_t eulXMSB = 0x1B;
uint8_t eulYLSB = 0x1C;
uint8_t eulYMSB = 0x1D;
uint8_t eulZLSB = 0x1E;
uint8_t eulZMSB = 0x1F;

uint8_t EXlsb = 0;
uint8_t EXmsb = 0;
uint8_t EYlsb = 0;
uint8_t EYmsb = 0;
uint8_t EZlsb = 0;
uint8_t EZmsb = 0;

uint8_t Mode[2];
uint8_t Units[2];

int16_t Xval = 0;
int16_t Yval = 0;
int16_t Zval = 0;

//Pressure Sensor Variables
uint8_t truncTemp;
uint8_t Depth;

typedef union _BYTE_TO_INT16 
{
    uint8_t Half[2];
    int16_t Whole;
}_BYTE_TO_INT16;

typedef union _BYTE_TO_UINT16 
{
    uint8_t Half[2];
    uint16_t Whole;
}_BYTE_TO_UINT16;

typedef union _BYTE_TO_UINT32 
{
    uint8_t Half[4];
    unsigned int Whole;
}_BYTE_TO_UINT32;

Watchdog &watchdog = Watchdog::get_instance(); 
//This part might be important to ensure function after power cycle
//Seems to work aight, might need to mess with timer duration
//Seems to be ok though...

int main()
{

    // thread_sleep_for(150); //added to see if there needed to be a small break before beginning the setup stuff...
    // hal_watchdog_init(1);
    // hal_watchdog_init(const watchdog_config_t *config);
    
    watchdog.start(1000);
    /***INITIALIZATIONS***/
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    
    HAL_Init();
    GPIO_Init();           
    //Timer Initialization for PWM outputs
    TIM4_Init();
    TIM9_Init();
    TIM10_Init();
    TIM11_Init();
    TIM13_Init();
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    
    //I2C1_Init();                 //Initialize I2C for sensors
    USART2_UART_Init();          //Initialize UART2 for the Serial Monitor
    USART3_UART_Init();          //Initialize UART3 for RS422 Coms
    /***END INITIALIZATIONS***/

    DMA_Start_Transmit();        //Start DMA Transmission on UART3 for RS422
    motorControlThread.start(motorControl);     //Start Motor Control (Init's as OFF)
    //packetParserThread.start(packetParser);     //Start Parsing of Received Wireless Packets 
/*
    //Mode Select Address and Value
    Mode[0] = 0x3D;
    Mode[1] = 0x0C;
    //Unit Select Address and Value
    Units[0] = 0x3B;
    Units[1] = 0x00;
    //Mode Select
    HAL_I2C_Master_Transmit(&hi2c1, 0x28<<1, Mode, 2, 100);
    thread_sleep_for(100);
    HAL_I2C_Master_Transmit(&hi2c1, 0x28<<1, Units, 2, 100);
    thread_sleep_for(100);

    //Call Once to initialize pressure sensor
    uint8_t resSeq = 30;
    HAL_I2C_Master_Transmit(&hi2c1, 0x76<<1, &resSeq, 1, 1000);
*/
    while(1)
    {
        //printf("L1:%d|L2:%d|LeftHatX:%d|LeftHatY:%d|RightHatX:%d|RightHatY:%d|R1:%d|R2:%d|Triangle:%d|Circle:%d|Cross:%d|Square:%d|\n", L1, L2, LeftHatX, LeftHatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square);
       
        //readIMUData();          //Gather IMU data for X, Y, Z position
        //readPressureSensor();   //Gather pressure sensor data for depth and temperature 
        //createTransmitPacket(); //Populate the sending packet with sensor data
        
        watchdog.kick();

       

        /***START FILTER***/
        //Filter for DMA buffer to select good packets with conection lost check
        if(startFilterFlag) //Flag set in DMA or UART idle line interrupts
        {
            //Sample Packet: "#99000000134111154132000000999#"
            memcpy(RS422_RxDataBuf, &RS422_RxData, 500);
            for(int i = 0; i < 500; i++) //Filter through 500 byte buffer and grab a packet
            {
                if(RS422_RxDataBuf[i] == '#' && (RS422_RxDataBuf[i+32] == '#')) //Check for a # at start and end of packet
                {
                    memcpy(subtext2, &RS422_RxDataBuf[i+1], 2); //Parse out the packet counter
                    sscanf(subtext2, "%d", &newPacketCount);    //Convert packet count to int

                    //ex. new = 5, old = 4 OR new = 1, old = 99
                    if(newPacketCount != oldPacketCount)        
                    {
                        memcpy(parseBuffer, &RS422_RxDataBuf[i], 32); //Move received data into a buffer to be parsed
                        oldPacketCount = newPacketCount;
                        motorsToggle = true;
                        HAL_UART_Transmit(&huart2, (uint8_t *)parseBuffer, 32, 100); //Send filtered packet over debug pin

                        //L1
                        memcpy(subtext3, &parseBuffer[3], 3);
                        subtext3[3] = '\0';
                        sscanf(subtext3, "%d", &L1);
                        //L2
                        memcpy(subtext3, &parseBuffer[6], 3);
                        subtext3[3] = '\0';
                        sscanf(subtext3, "%d", &L2);
                        //LeftHatX
                        memcpy(subtext3, &parseBuffer[9], 3);
                        subtext3[3] = '\0';
                        sscanf(subtext3, "%d", &LeftHatX);
                        //LeftHatY
                        memcpy(subtext3, &parseBuffer[12], 3);
                        subtext3[3] = '\0';
                        sscanf(subtext3, "%d", &LeftHatY);
                        //RightHatX
                        memcpy(subtext3, &parseBuffer[15], 3);
                        subtext3[3] = '\0';
                        sscanf(subtext3, "%d", &RightHatX);
                        //RightHatY
                        memcpy(subtext3, &parseBuffer[18], 3);
                        subtext3[3] = '\0';
                        sscanf(subtext3, "%d", &RightHatY);
                        //R1
                        memcpy(subtext1, &parseBuffer[19], 1);
                        subtext1[1] = '\0';
                        sscanf(subtext1, "%d", &R1);
                        //R2
                        memcpy(subtext1, &parseBuffer[20], 1);
                        subtext1[1] = '\0';
                        sscanf(subtext1, "%d", &R2);
                        //Triangle
                        memcpy(subtext1, &parseBuffer[21], 1);
                        subtext1[1] = '\0';
                        sscanf(subtext1, "%d", &Triangle);
                        //Circle
                        memcpy(subtext1, &parseBuffer[22], 1);
                        subtext1[1] = '\0';
                        sscanf(subtext1, "%d", &Circle);           
                        //Cross
                        memcpy(subtext1, &parseBuffer[23], 1);
                        subtext1[1] = '\0';
                        sscanf(subtext1, "%d", &Cross);
                        //Square
                        memcpy(subtext1, &parseBuffer[24], 1);
                        subtext1[1] = '\0';
                        sscanf(subtext1, "%d", &Square);  
                        thread_sleep_for(5); //Relieve time for CPU to handle other processes
                    }
                    //If new == old conncection has been lost
                    else 
                    {
                        motorsToggle = false; //Turn off all motors
                    }
                }
            }
        }
        /***END FILTER***/
    }
}

/***THREADS***/

void motorControl()
{
    //Thread that controls PWM signal outputs, uses data from parse packet thread
    //New PWM: 400Hz | 2.5ms | 0 - 2500 | Max: 1860us | Stop: 1500us | Min: 1060us
    //Motor control operation: Initialize all ESC's with stop of 1500us for 4 seconds and populate large arrays with stop as well.
    //In while loop, check if motorsoff flag is triggered, to shut all motors off. (Ex. lost connection)
    //Maps the parsed variables from controls to us values into index 0 of each array for the PWM outputs.
    //Shifts all arrays to the right, so updated value is now index 1.
    //Takes the average of the array, and writes that to the PWM output.
    //PWM is smoothed as the moving average ensures if full throttle is held, it takes time to populate the array with max values.

    //Define max and min power
    int maxPower = 1650; 
    int minPower = 1350;
    int stopPower = 1500;

    const int n = 200; //Array size for averaging 
    //Initialize Arrays
    vector<int> motor1DCarray(n); 
    vector<int> motor2DCarray(n); 
    vector<int> motor3DCarray(n); 
    vector<int> motor4DCarray(n); 
    vector<int> motor5DCarray(n); 
    vector<int> motor6DCarray(n); 
    //Initalize Averages at stop
    int motor1DCavg = stopPower; 
    int motor2DCavg = stopPower;
    int motor3DCavg = stopPower;
    int motor4DCavg = stopPower;
    int motor5DCavg = stopPower;
    int motor6DCavg = stopPower;

    /***FILL ARRAYS***/
    for(int i = 0; i < n; i++)
    {
        motor1DCarray[i] = stopPower; //Fill arrays with stop power
    }
    for(int i = 0; i < n; i++)
    {
        motor2DCarray[i] = stopPower;
    }
    for(int i = 0; i < n; i++)
    {
        motor3DCarray[i] = stopPower;
    }
    for(int i = 0; i < n; i++)
    {
        motor4DCarray[i] = stopPower;
    }
    for(int i = 0; i < n; i++)
    {
        motor5DCarray[i] = stopPower;
    }
    for(int i = 0; i < n; i++)
    {
        motor6DCarray[i] = stopPower;
    }
    /***WRITE TO PWM OUT***/
    TIM4->CCR1 =  stopPower;  //Rear  L
    TIM4->CCR2 =  stopPower;  //Front R
    TIM9->CCR2 =  stopPower;  //Rear  R
    TIM13->CCR1 = stopPower;  //Front L
    TIM11->CCR1 = stopPower;  //Back  Z
    TIM10->CCR1 = stopPower;  //Front Z
    
    thread_sleep_for(2000); //Delay 4 seconds to allow ESC's to initialize
    /***MAIN MOTOR CONTROL LOOP***/
    while (1)
    {
        watchdog.kick();
        if (!motorsToggle) //Turn all motors OFF case
        {
            /***WRITE TO PWM OUT***/
            TIM4->CCR1 =  stopPower;  //Rear  L
            TIM4->CCR2 =  stopPower;  //Front R
            TIM9->CCR2 =  stopPower;  //Rear  R
            TIM13->CCR1 = stopPower;  //Front L
            TIM11->CCR1 = stopPower;  //Back  Z
            TIM10->CCR1 = stopPower;  //Front Z
        }

        /***MAP ARRAYS***/
        //
        if (L1 > 5 && L2 < 5)
        {
            motor5DCarray[0] = customMap(L1, 0, 255, stopPower, maxPower); 
            motor6DCarray[0] = customMap(L1, 0, 255, stopPower, maxPower);
        }
        else if (L2 > 5 && L1 < 5)
        {
            motor5DCarray[0] = customMap(L2, 0, 255, stopPower, minPower);
            motor6DCarray[0] = customMap(L2, 0, 255, stopPower, minPower);   
        }
        else
        {
            motor5DCarray[0] = stopPower; 
            motor6DCarray[0] = stopPower;
        }

        if(LeftHatY > 129) //129 - 255 Reverse
        {
            motor1DCarray[0] = customMap(LeftHatY, 129, 255, stopPower, maxPower); 
            motor4DCarray[0] = customMap(LeftHatY, 129, 255, stopPower, minPower);
        }
        else if(LeftHatY < 127)
        {
            motor1DCarray[0] = customMap(LeftHatY, 0, 127, minPower, stopPower); 
            motor4DCarray[0] = customMap(LeftHatY, 0, 127, maxPower, stopPower);
        }
        else
        {
            motor1DCarray[0] = stopPower;
            motor4DCarray[0] = stopPower;
        }

        if(RightHatY > 129)
        {
            motor2DCarray[0] = customMap(RightHatY, 129, 255, stopPower, minPower); 
            motor3DCarray[0] = customMap(RightHatY, 129, 255, stopPower, minPower);
        }
        else if(RightHatY < 127)
        {
            motor2DCarray[0] = customMap(RightHatY, 0, 127, maxPower, stopPower); 
            motor3DCarray[0] = customMap(RightHatY, 0, 127, maxPower, stopPower);
        }
        else 
        {
            motor2DCarray[0]= stopPower;
            motor3DCarray[0] = stopPower;
        }

        /***SHIFT ARRAYS***/
        for(int i=n; i>=0; i--) 
        {
            motor1DCarray[i+1] = motor1DCarray[i]; 
        }

        for(int i=n; i>=0; i--)
        {
            motor2DCarray[i+1] = motor2DCarray[i]; 
        }

        for(int i=n; i>=0; i--)
        {
            motor3DCarray[i+1] = motor3DCarray[i]; 
        }
        
        for(int i=n; i>=0; i--)
        {
            motor4DCarray[i+1] = motor4DCarray[i]; 
        }

        for(int i=n; i>=0; i--)
        {
            motor5DCarray[i+1] = motor5DCarray[i]; 
        }

        for(int i=n; i>=0; i--)
        {
            motor6DCarray[i+1] = motor6DCarray[i]; 
        }

        /***AVERAGE ARRAYS***/
        for(int i = 1; i<n; ++i)
        {
            motor1DCavg += motor1DCarray[i];
        }
        motor1DCavg = motor1DCavg/n; 
        for(int i = 1; i<n; ++i)
        {
            motor2DCavg += motor2DCarray[i];
        }
        motor2DCavg = motor2DCavg/n;
        for(int i = 1; i<n; ++i)
        {
            motor3DCavg += motor3DCarray[i];
        }
        motor3DCavg = motor3DCavg/n;
        for(int i = 1; i<n; ++i)
        {
            motor4DCavg += motor4DCarray[i];
        }
        motor4DCavg = motor4DCavg/n;
        for(int i = 1; i<n; ++i)
        {
            motor5DCavg += motor5DCarray[i];
        }
        motor5DCavg = motor5DCavg/n;
        for(int i = 1; i<n; ++i)
        {
            motor6DCavg += motor6DCarray[i];
        }
        motor6DCavg = motor6DCavg/n;

        //***WRITE TO PWM OUT***/
        TIM4->CCR1 =  motor1DCavg;  //Rear  L
        TIM4->CCR2 =  motor2DCavg;  //Front R
        TIM9->CCR2 =  motor3DCavg;  //Rear  R
        TIM13->CCR1 = motor4DCavg;  //Front L
        TIM11->CCR1 = motor5DCavg;  //Back  Z
        TIM10->CCR1 = motor6DCavg;  //Front Z
    }
}

/***ACTIVE FUNCTIONS***/
void readIMUData()
{
    //Read IMU Data
    union _BYTE_TO_INT16 EX;
    union _BYTE_TO_INT16 EY;
    union _BYTE_TO_INT16 EZ;

    //Read Euler Angles
    //Function takes address of LSB and MSB Euler data registers for a specific axis as well as a location to store the data
    getIMUreg(eulXLSB, eulXMSB, &EXlsb, &EXmsb);
    EX.Half[0] = EXlsb;
    EX.Half[1] = EXmsb;
    getIMUreg(eulYLSB, eulYMSB, &EYlsb, &EYmsb);
    EY.Half[0] = EYlsb;
    EY.Half[1] = EYmsb;
    getIMUreg(eulZLSB, eulZMSB, &EZlsb, &EZmsb);
    EZ.Half[0] = EZlsb;
    EZ.Half[1] = EZmsb;

    Xval = customMap(abs(EX.Whole), 0, 5900, 0, 200);
    Yval = customMap(abs(EY.Whole), -1440, 1440, 0, 200);
    Zval = customMap(abs(EZ.Whole), -2880, 2880, 0, 200);

    //printf("IMU Data: X = %d Y = %d Z = %d \n", Xval, Yval, Zval);
}

void readPressureSensor()
{
    union _BYTE_TO_UINT32 U32;
    //     union _BYTE_TO_UINT16 U16;
    //  uint8_t C5Add = 0xAA;
    //  uint8_t C6Add = 0xAC;
    //  uint8_t C1Add = 0xA2;
    //  uint8_t C2Add = 0xA4;
    //  uint8_t C3Add = 0xA6;
    //  uint8_t C4Add = 0xA8;
    uint8_t getPres = 0x48;
    uint8_t getTemp = 0x58;
    uint8_t readSens = 0x00;
    uint8_t receiveBuff[3];
    //  uint8_t receiveBuf[2];
    uint16_t C1 = 33647;
    uint16_t C2 = 34302;
    uint16_t C3 = 20176;
    uint16_t C4 = 21815;
    uint16_t C5 = 26721;
    uint16_t C6 = 26600;
    uint32_t uncompTemp = 0;
    uint32_t uncompPres = 0;
    int32_t dT = 0;
    float Temp = 0;
    double Pres = 0;
    int64_t OFF = 0;
    int64_t SENS = 0;

    HAL_I2C_Master_Transmit(&hi2c1, 0x76<<1, &getTemp, 1, 1000);
    thread_sleep_for(20);
    HAL_I2C_Master_Transmit(&hi2c1, 0x76<<1, &readSens, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c1, 0x76<<1, &receiveBuff[0], 3, 1000);
    U32.Half[3] = 0;
    U32.Half[0] = receiveBuff[2];
    U32.Half[1] = receiveBuff[1];
    U32.Half[2] = receiveBuff[0];
    uncompTemp = U32.Whole;

    dT = (uncompTemp - (C5 * 256.0));
    Temp = ((2000 + (dT * C6 / 8388608.0))/100.0);
    printf("Temp = %0.2f \n", Temp);
    HAL_I2C_Master_Transmit(&hi2c1, 0x76<<1, &getPres, 1, 1000);
    thread_sleep_for(20);
    HAL_I2C_Master_Transmit(&hi2c1, 0x76<<1, &readSens, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c1, 0x76<<1, &receiveBuff[0], 3, 1000);
    U32.Half[3] = 0;
    U32.Half[0] = receiveBuff[2];
    U32.Half[1] = receiveBuff[1];
    U32.Half[2] = receiveBuff[0];
    uncompPres = U32.Whole;

    int64_t tOff = (int64_t)(C4 / 128.0 * dT);
    uint32_t tOff1 = C2 * 65536;
    OFF = ((int64_t)tOff1 + tOff);
    SENS = (C1 * 32768 + (C3 * (int64_t)dT) /256);
    Pres = (((uncompPres * SENS / 2097152.0 - OFF) / 8192.0) /10.0);

    truncTemp = (uint8_t)Temp;
    Depth = customMap(Pres, 1000, 10000, 0, 100);

    //printf("\n \n trunc: %d depth: %d \n \n", truncTemp, Depth);
    //printf("Pressure = %0.0f mbar \n", Pres);
}

void createTransmitPacket()
{
    //Receive IMU data make packet 
    //RS422_TxData fill with sensor shit
    //    IMX IMY IMZ Dep Tem
    //#99 000 000 000 000 00
    //#99 000 000 000 000 000 0 0 0 0 0 0 0 0 999#

    uint8_t arrayBuffer[32] = "#9900000000000000000000000999#";
    int buffer = Xval;
    for(int i = 5; i < 3; i--)
    {
        arrayBuffer[i] = buffer % 10;
        buffer /= 10;
    }

    buffer = Yval;
    for(int i = 8; i < 6; i--)
    {
        arrayBuffer[i] = buffer % 10;
        buffer /= 10;
    }

    buffer = Zval;
    for(int i = 11; i < 9; i--)
    {
        arrayBuffer[i] = buffer % 10;
        buffer /= 10;
    }

    buffer = Depth;
    for(int i = 14; i < 12; i--)
    {
        arrayBuffer[i] = buffer % 10;
        buffer /= 10;
    }

    buffer = truncTemp;
    for(int i = 16; i < 14; i--)
    {
        arrayBuffer[i] = buffer % 10;
        buffer /= 10;
    }

    buffer = transmitPacketCount++;
    for(int i = 1; i < 2; i--)
    {
        arrayBuffer[i] = buffer % 10;
        buffer /= 10;
    }

    memcpy(RS422_TxData, &arrayBuffer, 32);
    DMA_Start_Transmit();
}

void getIMUreg(uint8_t register_pointerLSB, uint8_t register_pointerMSB, uint8_t* receive_bufferLSB, uint8_t* receive_bufferMSB)
{
    // 0x28 is device address shifted 1 left, register pointer is the location of the register you want to read, 1 is the size of the register pointer
    HAL_I2C_Master_Transmit(&hi2c1, 0x28<<1, &register_pointerLSB, 1, 100);

    // Data from register transmitted to previously is sent to receive buffer
    HAL_I2C_Master_Receive(&hi2c1, 0x28<<1, receive_bufferLSB, 1, 100);

    thread_sleep_for(50);

    HAL_I2C_Master_Transmit(&hi2c1, 0x28<<1, &register_pointerMSB, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x28<<1, receive_bufferMSB, 1, 100);   
}

void DMA_Start_Transmit(void)
{
    //Disable the channel or stream to be reconfigured before Tx
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
    //Configure DMA memory addresses for UART to grab from
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)RS422_TxData);
    //Configure DMA length (number of bytes to transmit)
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, ARRAY_LEN(RS422_TxData));
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, 32);
    //Enabe UART DMA Tx Request
    LL_USART_EnableDMAReq_TX(USART3); //this line freeze
    //Enable UART Tx Direction
    LL_USART_EnableDirectionTx(USART3); //Not sure if needed 
    //Clear all flags
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    //Enable DMA stream or channel to start tranmission
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3); 
}

int customMap(int x, int in_min, int in_max, int out_min, int out_max) 
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
    //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);

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

    //Configure GPIO pins for I2C
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
}

static void I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 38;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)RS422_RxData);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ARRAY_LEN(RS422_RxData));

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
    NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));//was 1
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

    //Enable USART3 and DMA Rx Stream 
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    LL_USART_Enable(USART3);
}

static void TIM4_Init(void)
{
    //APB1 @ 42MHz - we want ~400Hz or 2.5ms
    //Formula: Timer Frequency = (Bus Clock / Prescaler) / Period
    //                          400Hz = (42MHz / 42) / 2500 
    //789Hz or 781 = (BusClk / 42) / 2500
    //400 = (82000000 /)

    __HAL_RCC_TIM4_CLK_ENABLE();

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 83;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 2500;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
    Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
    Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
    Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
    Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
    Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
    Error_Handler();
    }
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void TIM9_Init(void)
{
    //APB2 @ 84MHz - we want ~400Hz or 2.5ms
    //Formula: Timer Frequency = (Bus Clock / Prescaler) / Period
    //                          400Hz = (84MHz / 84) / 2500 
    __HAL_RCC_TIM9_CLK_ENABLE();

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    htim9.Instance = TIM9;
    htim9.Init.Prescaler = 167;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = 2500;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
    {
    Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
    {
    Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
    {
    Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
    Error_Handler();
    }

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

static void TIM10_Init(void)
{
    //APB2 @ 84MHz - we want ~400Hz or 2.5ms
    //Formula: Timer Frequency = (Bus Clock / Prescaler) / Period
    //                          400Hz = (84MHz / 84) / 2500 
    __HAL_RCC_TIM10_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 167;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 2500;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
    Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
    {
    Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
    Error_Handler();
    }
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

static void TIM11_Init(void)
{
    //APB2 @ 84MHz - we want ~400Hz or 2.5ms
    //Formula: Timer Frequency = (Bus Clock / Prescaler) / Period
    //                          400Hz = (84MHz / 84) / 2500 
    __HAL_RCC_TIM11_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 167;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 2500;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
    {
    Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
    {
    Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
    Error_Handler();
    }

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

static void TIM13_Init(void)
{
    //APB1 @ 42MHz - we want ~400Hz or 2.5ms
    //Formula: Timer Frequency = (Bus Clock / Prescaler) / Period
    //                          400Hz = (42MHz / 42) / 2500 
    __HAL_RCC_TIM13_CLK_ENABLE();
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    htim13.Instance = TIM13;
    htim13.Init.Prescaler = 83;
    htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim13.Init.Period = 2500;
    htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
    {
    Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
    {
    Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
    Error_Handler();
    }

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/***INTERRUPT HANDLERS***/
void DMA1_Stream1_IRQHandler(void) 
{
    // Check transfer-complete interrupt for Rx
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) 
    {
        LL_DMA_ClearFlag_TC1(DMA1);             //Clear transfer complete flag 
        startFilterFlag = true;
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

void Error_Handler(void)
{
    //User can add his own implementation to report the HAL error return state
    __disable_irq();
    while (1)
    {
    }
}