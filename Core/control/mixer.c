/** @file 		mixer.c
 *  @brief
 *  	This file takes the output of the PID controller and assigns values to the motors.
 *
 *  @author 	Khoa Nguyen
 *  @date 		28 SAT 2024
 */

/* Includes */
#include "board.h"

/* Global Variables */
uint8_t numberMotor = 4;
uint16_t throttleCmd;
int16_t steerCmd, speedCmd;
int16_t motor_temp[4];

/** @brief Mixes the values from the PID controller and assigns values to the motors.
 *
 *  @return Void.
 */
void
mixTable(void)
{
	uint8_t i;

	if(armed == true)
	{
		Motors_SetPWM(&motors[RIGHT_BACK_MOTOR] , 	PIDMIXFLIGHT( -1.0f,  1.0f,  1.0f, 1.0f ));      // Rear Right  CW
		Motors_SetPWM(&motors[RIGHT_FRONT_MOTOR] , 	PIDMIXFLIGHT( -1.0f, -1.0f, -1.0f, 1.0f ));      // Front Right CCW
		Motors_SetPWM(&motors[LEFT_BACK_MOTOR] , 	PIDMIXFLIGHT(  1.0f,  1.0f, -1.0f, 1.0f ));      // Rear Left   CCW
		Motors_SetPWM(&motors[LEFT_FRONT_MOTOR] , 	PIDMIXFLIGHT(  1.0f, -1.0f,  1.0f, 1.0f ));      // Front Left  CW

		float maxDeltaThrottle;
		float minDeltaThrottle;
		float deltaThrottle;

		maxDeltaThrottle = MAX_PWM - rxCommands[THROTTLE];
		minDeltaThrottle = rxCommands[THROTTLE] - MIN_PWM;
		deltaThrottle    = (minDeltaThrottle < maxDeltaThrottle) ? minDeltaThrottle : maxDeltaThrottle;

		for (i = 0; i < MOTOR_COUNT; i++)
			motors[i] = constrain(motors[i], rxCommands[THROTTLE] - deltaThrottle, rxCommands[THROTTLE] + deltaThrottle);
	}
	else
	{
		motors[RIGHT_BACK_MOTOR] = 0;
		motors[RIGHT_FRONT_MOTOR] = 0;
		motors[LEFT_BACK_MOTOR] = 0;
		motors[LEFT_FRONT_MOTOR] = 0;
	}
}
