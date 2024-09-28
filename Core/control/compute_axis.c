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
    float CommandtoAngle;

    CommandtoAngle = ANGLE_SCALING(rxCommands[ROLL]); 
    error = CommandtoAngle - imu.angleRoll;
    ratePID[ROLL] = angle_PID_drone(&pid_roll, error, imu.mpu.rateRoll, 0.7, dt500Hz);

    CommandtoAngle = ANGLE_SCALING(rxCommands[PITCH]);  
    error = CommandtoAngle - imu.anglePitch;
    ratePID[PITCH] = angle_PID_drone(&pid_pitch, error, imu.mpu.ratePitch, 0.7, dt500Hz);

    CommandtoAngle = ANGLE_SCALING(rxCommands[YAW]);
    error = CommandtoAngle - imu.mpu.rateYaw;
    ratePID[YAW] = base_PID_calc(&pid_yaw, error, imu.mpu.rateYaw, 0.7, dt500Hz);

    ///////////////////////////////////

    throttleCmd = rxCommands[THROTTLE];
}

//////////////////////////////////////////////////////////////////////////////
