/*
 * mpu6050.h
 *
 *  Created on: Jun 30, 2024
 *      Author: Nguyen Tran Dang Khoa
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <math.h>
#include <stdint.h>
#include "gy_bmp280.h"
#include ".\board.h"
/* 										User notes  						   */
/* Must to set up interrupt or anything for sampling time equal to your define */
/*******************************************************************************/
/* User Configurations */

#define MPU6050_I2C 				MY_MPU6050
// calibration to reach 1g for accelerometer
#define CALIB_Ax_VALUE 0.07
#define CALIB_Ay_VALUE -0.02
#define CALIB_Az_VALUE 0.01

#define SAMPLING_TIME 	0.002 	// s -> 500Hz
#define STD_DEV_MEA		3		// degree per second
#define STD_DEV_EST 	4  		// degree per second
#define E_est SAMPLING_TIME*SAMPLING_TIME*STD_DEV_EST*STD_DEV_EST
#define STD_DEV_BARO 30 // cm
#define STD_DEV_ACC 10 // cm/s^2

/* END User Configurations */

/* MPU6050 typedefs and define */
#define MPU6050_ADDR 0xD0
#define RAD_TO_DEG 57.295779513082320876798154814105
#define PHYSICAL_CONVERT_GYRO 65.5
#define PHYSICAL_CONVERT_ACCEL 4096
#define TIME_REF 2000
#define i2c_timeout 100

typedef struct
{
    double angle;
    // User define initial estimate error
    double estimate_error;
} Kalman_1D_t;


typedef struct {
    double S[2][1];     // State estimate vector (vertical velocity, altitude)
    double P[2][2];    // prediction uncertainty vector
    double F[2][2];    // state transition matrix
    double H[1][2];    // Observation matrix
    double R[1][1];    // Measurement uncertainty
    double G[2][1];   // control matrix
    // double K[2][1];      // Kalman gain
    double Q[2][2];   // process uncertainty
} Kalman_2D_t;

typedef struct
{
	int16_t X_raw;
	int16_t Y_raw;
	int16_t Z_raw;

	double rateRoll;
	double ratePitch;
	double rateYaw;
	double ref_point_X;
	double ref_point_Y;
	double ref_point_Z;

	double Ax;
	double Ay;
	double Az;
} MPU_6050;
/* END MPU6050 typedefs and define */

/* MPU6050 functions */
/**
  * @brief  Init mpu6050 for drone
  * @param  mpu: pointer to MPU_6050 struct
  * @retval 0 if success, 1 if failed
*/
uint8_t mpu6050_init(MPU_6050 * mpu);

/**
  * @brief  get reference point for mpu6050
  * @param  mpu: pointer to MPU_6050 struct
  * @retval None
*/
void get_ref_point(MPU_6050 * mpu);

/**
  * @brief  Concatenate two half data to full data
  * @param  mpu: pointer to MPU_6050 struct
  * @param  raw_data: array of 2 half data
  * @retval None
*/
void concat_raw_data(MPU_6050 * mpu, uint8_t * raw_data);

/**
  * @brief  Update all data of mpu6050 (gyro, accel, angle)
  * @param  mpu: pointer to MPU_6050 struct
  * @retval None
  * 
*/void mpu6050_update_all(MPU_6050 * mpu);

/* END MPU6050 functions */

/* MPU6050 Registers */
#define WHO_AM_I 		0x75
#define CONFIG 			0x1A
#define PWR_MGMT_1 		0x6B
#define SMPLRT_DIV 		0x19
#define ACCEL_CONFIG 	0x1C
#define ACCEL_XOUT_H 	0x3B
#define TEMP_OUT_H 		0x41
#define GYRO_CONFIG 	0x1B
#define GYRO_XOUT_H 	0x43
/* END MPU6050 Registers */

#endif /* INC_MPU6050_H_ */