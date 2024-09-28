/*
 * imu.h
 *
 *  Created on: Sep 6, 2024
 *      Author: Nguyen Tran Dang Khoa
 */


#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "mpu6050.h"
#include "gy_bmp280.h"

#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3

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

typedef struct {
    double angleRoll;
	  double anglePitch;

    double vertical_velocity;
    double inertial_z;
    double altitude;

	  Kalman_1D_t Kalman_X;
	  Kalman_1D_t Kalman_Y;

    Kalman_2D_t Kalman_2D;

    MPU_6050 mpu;
    BMP280 bmp;
} IMU;


/**
  * @brief  Intialize IMU including MPU6050 and BMP280
  * @param  imu: pointer to IMU struct
  * @retval None
*/
void imu_init(IMU * imu);

/**
  * @brief  Update MPU6050 data (angle, rate, accel, inertial_z)
  * @param  imu: pointer to IMU struct
  * @retval None
*/
void imu_update_mpu(IMU * imu);

/**
  * @brief  Update BMP280 data (altitude)
  * @param  imu: pointer to IMU struct
  * @retval None
*/
void imu_update_bmp(IMU * imu);

/**
  * @brief  Update attitude of drone (angle, vertical velocity, altitude)
  * @param  imu: pointer to IMU struct
  * @retval None
*/
void imu_update_attitude(IMU * imu);

/**
  * @brief  1D Kalman filter for MPU6050 to calculate angle
  * @param  Kalman: pointer to Kalman struct
  * @param  newAngle: new angle (calculate from accel) from MPU6050
  * @param  newRate: new rate (calculate from gyro) from MPU6050
*/
double Kalman_getAngle(Kalman_1D_t * Kalman, double newAngle, double newRate);

/**
  * @brief  2D Kalman filter for MPU6050 and BMP280 to calculate attitude and vertical velocity
  * @param  filter: pointer to Kalman_2D struct
  * @param  dt: sampling time
*/
void Kalman_2D_init(Kalman_2D_t * filter, double dt);

/**
  * @brief  1D Kalman filter for MPU6050 to calculate angle
  * @param  filter: pointer to Kalman_1D struct
*/
void Kalman_1D_init(Kalman_1D_t * filter);

/**
  * @brief  2D Kalman filter for MPU6050 and BMP280 to calculate attitude and vertical velocity
  * @param  kf: pointer to Kalman_2D struct
  * @param  alt: altitude from BMP280
  * @param  iner_z: vertical velocity from MPU6050
*/
void kalman_attitude_filter(Kalman_2D_t * kf, double alt, double iner_z);
#endif /* INC_IMU_H_ */