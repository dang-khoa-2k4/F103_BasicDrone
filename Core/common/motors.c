/** @file 		config.c
 *  @brief
 *  	This file contains functions to initialize the ESC or change modes
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */
#include "board.h"

void Motors_Init(void)
{
    HAL_TIM_PWM_Start(RIGHT_MOTOR_PART, RBMotor);
    HAL_TIM_PWM_Start(RIGHT_MOTOR_PART, RFMotor);
    HAL_TIM_PWM_Start(LEFT_MOTOR_PART, LBMotor);
    HAL_TIM_PWM_Start(LEFT_MOTOR_PART, LFMotor);

    motors[1] = MIN_PWM;
    motors[2] = MIN_PWM;
    motors[3] = MIN_PWM;
    motors[4] = MIN_PWM;
}

void Motors_SetPWM(speed_t motor, float pwm)
{
    
} 

void Motors_SetAllPWM(speed_t * motors, float pwm1, float pwm2, float pwm3, float pwm4);