/** @file 		config.c
 *  @brief
 *  	This file contains functions to initialize the ESC or change modes
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */
#include "board.h"

speed_t motors[MOTOR_COUNT]; // 0 -> 3 : RB RF LB LF

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

void Motors_SetPWM(speed_t * motor, float pwm)
{
    *motor = constrain(pwm, MIN_PWM, MAX_PWM);
} 

void Motors_SetAllPWM(float pwm1, float pwm2, float pwm3, float pwm4)
{
    motors[1] = constrain(pwm1, MIN_PWM, MAX_PWM);
    motors[2] = constrain(pwm2, MIN_PWM, MAX_PWM);
    motors[3] = constrain(pwm3, MIN_PWM, MAX_PWM);
    motors[4] = constrain(pwm4, MIN_PWM, MAX_PWM);
}

void Motors_Run(void)
{
    __HAL_TIM_SET_COMPARE(RIGHT_MOTOR_PART, RBMotor, motors[1]);
    __HAL_TIM_SET_COMPARE(RIGHT_MOTOR_PART, RFMotor, motors[2]);
    __HAL_TIM_SET_COMPARE(LEFT_MOTOR_PART, LBMotor, motors[3]);
    __HAL_TIM_SET_COMPARE(LEFT_MOTOR_PART, LFMotor, motors[4]);
}