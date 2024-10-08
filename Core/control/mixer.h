/** @file 		mixer.c
 *  @brief
 *  	This file takes the output of the PID controller and assigns values to the motors.
 *
 *  @author 	Khoa Nguyen
 *  @date 		28 SAT 2024
 */

#ifndef __MIXER_H__
#define __MIXER_H__

#include <stdint.h>

/* Defines */
#define PIDMIXFLIGHT(X,Y,Z,T) 	(ratePID[ROLL] * (X) + ratePID[PITCH] * (Y) + ratePID[YAW] * (Z) + throttleCmd * (T))

#define THROTTLE_DEADBAND		400
#define THROTTLE_DEADBAND_SLOPE	((4000 - THROTTLE_DEADBAND)/4000.0f)

/* Global Variables */
extern uint8_t numberMotor;
extern uint16_t throttleCmd;
extern int16_t motor_temp[4];

/* Function Prototypes */
void mixTable(void);

#endif /* __MIXER_H__ */
