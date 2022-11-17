Walrus ROV
STM32F4 ROV Controller
02/11/2022
Alexander, Tony, Clinton
PS3 Controls Packet Format: 
0 12 345 678 901 234 567 890 1  2  3  4  5  6  789 0
"# 99 000 000 000 000 000 000 0  0  0  0  0  0  999 #" = 31
Pound, Count, L1, L2, LeftHatX, LefthatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square, Checksum, Pound
//Motor Pin Initializations:
PwmOut motor1(PB_7);       //Motor: Front Z
PwmOut motor2(PB_6);       //Motor: Rear Z
PwmOut motor3(PE_6);       //Motor: Front Right
PwmOut motor4(PF_8);       //Motor: Front Left
PwmOut motor5(PF_7);       //Motor: Back Right
PwmOut motor6(PF_6);       //Motor: Back Left

RESORUCES:
https://os.mbed.com/users/Owen/code/nRF24L01P/
https://os.mbed.com/cookbook/nRF24L01-wireless-transceiver
https://forums.mbed.com/t/hitchhikers-guide-to-printf-in-mbed-6/12492
https://controllerstech.com/how-to-setup-uart-using-registers-in-stm32/
https://arduino.ua/docs/AfroESC30A.pdf
https://os.mbed.com/handbook/PwmOut
https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
https://os.mbed.com/docs/mbed-os/v6.15/apis/thread.html
https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123
https://os.mbed.com/users/belloula/code/mbed_blinko//file/84c281baac9c/main.cpp/
