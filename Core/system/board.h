#ifndef __BOARD_H_
#define __BOARD_H_

#include "main.h"
#include "battery.h"
#include "buzzer.h"
#include "led.h"
#include "motors.h"
#include "scheduler.h"
#include "nRF24L01.h"
#include "hmc5883l.h"
// #include "mpu6050.h"
// #include "gy_bmp280.h"
#include "imu.h"
#include "PID.h"
#include "config.h"
#include "communicate.h"
#include "utilities.h"
#include "mixer.h"
#include "compute_axis.h"

extern I2C_HandleTypeDef 		hi2c1;
extern SPI_HandleTypeDef 		hspi1;
// extern ADC_HandleTypeDef 		hadc1;
extern TIM_HandleTypeDef        htim1;
extern TIM_HandleTypeDef        htim2;

extern uint8_t armed;

#define USE_IMU
#define USE_BMP
#define USE_MPU
#define USE_RX
#define USE_TX
#define USE_LED
#define USE_MOTOR
#define RUN
#define USE_BATTERY
// #define PRINT_LOG

// Define all the peripherals used in the project

// LED
#define led_0_Pin           LL_GPIO_PIN_13
#define led_0_GPIO_Port     GPIOC
#define led_1_Pin           LL_GPIO_PIN_14
#define led_1_GPIO_Port     GPIOB
#define led_2_Pin           LL_GPIO_PIN_15
#define led_2_GPIO_Port     GPIOB	

// BUZZER

// I2C
#define MY_BMP280  &hi2c1
#define MY_MPU6050 &hi2c1
#define MY_HMC5883L &hi2c1
// SPI
#define MY_NRF24L01 &hspi1
#define MY_NRF24L01_CE_PIN_PORT             GPIOB
#define MY_NRF24L01_CE_PIN_NUMBER           LL_GPIO_PIN_1
#define MY_NRF24L01_IRQ_PIN_PORT            GPIOA
#define MY_NRF24L01_IRQ_PIN_NUMBER          LL_GPIO_PIN_1
#define MY_NRF24L01_CS_PIN_PORT             GPIOB
#define MY_NRF24L01_CS_PIN_NUMBER           LL_GPIO_PIN_0

// TIM
#define RIGHT_MOTOR_PART &htim1
#define RBMotor TIM_CHANNEL_1
#define RFMotor TIM_CHANNEL_2
#define LEFT_MOTOR_PART &htim2
#define LBMotor TIM_CHANNEL_3
#define LFMotor TIM_CHANNEL_4
// ADC
// #define BATT_ADC        &hadc1
// UART
#define MY_UART       USART1

#define CONTROL_FREQ 500 // Hz
#define CHECK_BATT_FREQ 10 // Hz
#define LOG_FREG 100 // Hz
#define LED_FREQ 2 // Hz

#define TIM_PERIOD .1 // 0.1 ms = 100 us
#define TIM_FREQ 10000 // 10 kHz
/* Private defines -----------------------------------------------------------*/




#endif /* __BOARD_H_ */