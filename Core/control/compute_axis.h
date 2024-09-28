/** @file 		compute_axis_commands.h
 *  @brief
 *  	This file creates the inputs to the PID controller and sends values to the mixer.
 *
 *  @author 	Khoa Nguyen
 *  @date 		28 SAT 2024
 */

#ifndef __COMPUTE_AXIS_COMMANDS_H__
#define __COMPUTE_AXIS_COMMANDS_H__

/* Defines */
#define dt500Hz 0.002f // equals 500 Hz = pwm period
/* Global Variables */
extern int16_t  ratePID[3];
extern IMU imu;

/* Function Prototypes */
void computeAxisCommands(void);

#endif /* __COMPUTE_AXIS_COMMANDS_H__ */
