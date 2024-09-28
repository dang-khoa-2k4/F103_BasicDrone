/** @file 		config.c
 *  @brief
 *  	This file configures all settings of the flight controller.
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */

#include "board.h"

PID_instance pid_roll;
PID_instance pid_pitch;
PID_instance pid_yaw;
PID_instance pid_inner_roll;
PID_instance pid_inner_pitch;

void Config_Init(void)
{
    // Initialize the IMU
    imu_init(&imu);

    // Initialize the motors
    Motors_Init();

    // Initialize the nRF24L01
    nrf24l01_rx_init(2, _1Mbps);

    // Initialize the PID
        // initialize inner PID
        
        PID_init(&pid_inner_roll, NULL, kP_inner_roll, kI_inner_roll, kD_inner_roll);
        PID_init(&pid_inner_pitch, NULL, kP_inner_pitch, kI_inner_pitch, kD_inner_pitch);

    PID_init(&pid_roll, &pid_inner_roll, kP_roll, kI_roll, kD_roll);
    PID_init(&pid_pitch, &pid_inner_pitch, kP_pitch, kI_pitch, kD_pitch);
    PID_init(&pid_yaw, NULL, kP_yaw, kI_yaw, kD_yaw);


}