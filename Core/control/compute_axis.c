/** @file 		compute_axis_commands.h
 *  @brief
 *  	This file creates the inputs to the PID controller and sends values to the mixer.
 *
 *  @author 	Khoa Nguyen
 *  @date 		28 SAT 2024
 */

/* Includes */
#include "board.h"

/* Global Variables */
IMU imu;
int16_t ratePID[3];

/** @brief Computes the commands that get sent to the mixer.
 *
 *  @return Void.
 */
void computeAxisCommands(void)
{
    float error;

    error = rxCommands[ROLL] - imu.angleRoll;
    ratePID[ROLL] = angle_PID_drone(&pid_roll, error, imu.mpu.rateRoll, 0.7, dt500Hz);

    error = rxCommands[PITCH] - imu.anglePitch;
    ratePID[PITCH] = angle_PID_drone(&pid_pitch, error, imu.mpu.ratePitch, 0.7, dt500Hz);

    error = rxCommands[YAW] - imu.mpu.rateYaw;
    ratePID[YAW] = base_PID_calc(&pid_yaw, error, imu.mpu.rateYaw, 0.7, dt500Hz);

    ///////////////////////////////////

    throttleCmd = rxCommands[THROTTLE];
}

//////////////////////////////////////////////////////////////////////////////
