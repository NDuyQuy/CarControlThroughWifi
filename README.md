# CarControlThroughWifi

#Description

Build a small car model, which have two motor control by L298N motor driver. And control it through wifi by blynk.cloud.
This car model has the following components: ESP8266 NodeMCU board, STM32F103RCT6 board, L298N motor driver, 2 motor, 2 wheel, a 12v power source.
#Workflow

The ESP8266 board is connect to the wifi and get the data from the blynk. then it send a string contain mode and data receive to the STM32F103RCT6 board by uart.
The uart catch the content, get the data and the mode and then set the correspond value to control the motor and its speed.
#Pin connection

##ESP8266

Using the board hardware Rx,Tx for UART.

##STM32F103RCT6

Using 
PA8 for PWM(speed_control) motor_1, and PA4 is GPIO for motor_1
PA9 for PWM(speed_control) motor_2, and PA5 is GPIO for motor_2

PC10 is for UART_TX(hardware)
PC11 is for UART_RX(hardware)