/** @file 		config.h
 *  @brief
 *  	This file contains functions to initialize the ESC or change modes
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */

#ifndef __MOTORS_H__
#define __MOTORS_H__

/* Defines */
#define MOTOR_COUNT			4
#define MIN_PWM				1000
#define MAX_PWM				2000
#ifdef __cplusplus
 extern "C" {
#endif
/* Global Enums */
typedef uint16_t speed_t;

speed_t motors[MOTOR_COUNT];

void Motors_Init(void);
void Motors_SetPWM(speed_t motor, float pwm);
void Motors_SetAllPWM(float pwm1, float pwm2, float pwm3, float pwm4);

#ifdef __cplusplus
}
#endif

#endif /* __MOTORS_H__ */