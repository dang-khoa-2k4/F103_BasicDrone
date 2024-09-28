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

extern speed_t motors[MOTOR_COUNT]; // 0 -> 3 : RB RF LB LF

/* Function Prototypes */
/** @brief Initialize the ESCs with the minimum PWM.
 *
 *  @param None
 * 
 *  @return None
 */
void Motors_Init(void);

/** @brief Set the PWM of a motor.
 *
 *  @param motor Pointer to the motor speed_t structure.
 *  @param pwm The pwm will be set for that motor.
 * 
 *  @return None
 */
void Motors_SetPWM(speed_t * motor, float pwm);

/** @brief Set the PWM of all motors.
 *
 *  @param pwm1 PWM for RB motor.
 *  @param pwm2 PWM for RF motor.
 *  @param pwm3 PWM for LB motor.
 *  @param pwm4 PWM for LF motor.
 * 
 *  @return None
 */
void Motors_SetAllPWM(float pwm1, float pwm2, float pwm3, float pwm4);

/** @brief Run the motors.
 *
 *  @param None
 * 
 *  @return None
 */
void Motors_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTORS_H__ */