/*
 * imu.c
 *
 *  Created on: Sep 6, 2024
 *      Author: Nguyen Tran Dang Khoa
 */

#include "board.h"

IMU imu;

// Function for matrix multiplication of 2x2 and 2x1 matrices
static void mat_mult_2x2_2x1(double result[2][1], double A[2][2], double B[2][1]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
}

// Function for matrix addition of 2x1 matrices
static void mat_add_2x1(double result[2][1], double A[2][1], double B[2][1]) {
    result[0][0] = A[0][0] + B[0][0];
    result[1][0] = A[1][0] + B[1][0];
}

// Function for matrix multiplication of 2x2 matrices
static void mat_mult_2x2(double result[2][2], double A[2][2], double B[2][2]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
}

void Kalman_2D_init(Kalman_2D_t * kf, double dt)
{
	// Initialize state estimate vector
    kf->S[0][0] = 0.0;  // Initial vertical velocity
    kf->S[1][0] = 0.0;  // Initial altitude
    
    // Initialize covariance matrix
    kf->P[0][0] = 0.0;
    kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;
    kf->P[1][1] = 0.0;

	// State transition matrix
    kf->F[0][0] = 1.0;
    kf->F[0][1] = SAMPLING_TIME;
    kf->F[1][0] = 0.0;
    kf->F[1][1] = 1.0;

    // Measurement matrix
    kf->H[0][0] = 1.0;
    kf->H[0][1] = 0.0;

	// control matrix
	kf->G[0][0] = 0.5 * dt * dt;
	kf->G[1][0] = dt;

    // Measurement noise covariance matrix
    kf->R[0][0] = STD_DEV_BARO * STD_DEV_BARO;

	// Q = G * G' * STD_DEV_ACC * STD_DEV_ACC
	kf->Q[0][0] = kf->G[0][0] * kf->G[0][0] * STD_DEV_ACC * STD_DEV_ACC;
	kf->Q[0][1] = kf->G[0][0] * kf->G[1][0] * STD_DEV_ACC * STD_DEV_ACC;
    kf->Q[1][0] = kf->G[1][0] * kf->G[0][0] * STD_DEV_ACC * STD_DEV_ACC; 
	kf->Q[1][1] = kf->G[1][0] * kf->G[1][0] * STD_DEV_ACC * STD_DEV_ACC;
}

void Kalman_1D_init(Kalman_1D_t * filter)
{
    filter->angle = 0.0;
    filter->estimate_error = 0.0;
}

void imu_init()
{
    mpu6050_init(&(imu.mpu));
    bmp280_setup_Drone(&(imu.bmp));
    bmp280_init(&(imu.bmp));

    Kalman_2D_init(&(imu.Kalman_2D), SAMPLING_TIME);
    Kalman_1D_init(&(imu.Kalman_X));
    Kalman_1D_init(&(imu.Kalman_Y));
}

double Kalman_getAngle(Kalman_1D_t * filter, double newAngle, double newRate)
{
	filter->angle += newRate * SAMPLING_TIME;
	filter->estimate_error += E_est;
	double kalman_gain = filter->estimate_error / (filter->estimate_error + STD_DEV_MEA * STD_DEV_MEA);
	filter->angle += kalman_gain * (newAngle - filter->angle);
	filter->estimate_error *= (1 - kalman_gain);
	return filter->angle;
}

static double get_inertial_z(IMU * imu)
{
	double inertial_z = (imu->mpu.Az * cos(imu->angleRoll * RAD_TO_DEG) * cos(imu->anglePitch * RAD_TO_DEG)) +
						(imu->mpu.Ay * sin(imu->angleRoll * RAD_TO_DEG) * cos(imu->anglePitch * RAD_TO_DEG)) +
						(-imu->mpu.Ax * sin(imu->anglePitch * RAD_TO_DEG));
	// convert to cm/s and return 
	return (inertial_z - 1) * 9.81 * 100;
}

void imu_update_mpu(IMU * imu)
{
    mpu6050_update_all(&(imu->mpu));
    // convert to physical angle for drone control
    imu->angleRoll  = atan( imu->mpu.Ay / sqrt(pow(imu->mpu.Az, 2) + pow(imu->mpu.Ax, 2))) * RAD_TO_DEG;
    imu->anglePitch = atan(-imu->mpu.Ax / sqrt(pow(imu->mpu.Az, 2) + pow(imu->mpu.Ay, 2))) * RAD_TO_DEG;
    if ((imu->anglePitch < -90 && imu->Kalman_Y.angle > 90)
        || (imu->anglePitch > 90 &&  imu->Kalman_Y.angle < -90))
    {
        imu->Kalman_Y.angle  = imu->anglePitch;
    }
    else {	imu->anglePitch = Kalman_getAngle(&(imu->Kalman_Y), imu->anglePitch, imu->mpu.ratePitch); }

    if (fabs(imu->anglePitch) > 90) { imu->mpu.rateRoll = -imu->mpu.rateRoll; }
    imu->angleRoll = Kalman_getAngle(&(imu->Kalman_X), imu->angleRoll, imu->mpu.rateRoll);

    // update vertical velocity without Kalman filter
    imu->inertial_z = get_inertial_z(imu);
}

void imu_update_bmp(IMU * imu)
{
    imu->altitude = bmp280_get_altitude(&(imu->bmp));
}

void imu_update_attitude()
{
    imu_update_mpu(&imu);
    imu_update_bmp(&imu);
    kalman_attitude_filter(&(imu.Kalman_2D), imu.altitude, imu.inertial_z);
    imu.altitude = imu.Kalman_2D.S[1][0];
    imu.vertical_velocity = imu.Kalman_2D.S[0][0];
}

void kalman_attitude_filter(Kalman_2D_t * kf, double alt, double iner_z) {
    double S_pred[2][1];  // Predicted state
    double P_pred[2][2];  // Predicted uncertainty
    double K[2][1];       // Kalman gain
    double temp1[2][2];   // Temporary matrices for calculation
    double temp2[1][1];   // Temporary matrix for calculation
    double innovation[1][1]; // Innovation (measurement - prediction)
    
    // 1. Predict the current state of the system: S(k) = F * S(k-1) + G * U(k)
    mat_mult_2x2_2x1(S_pred, kf->F, kf->S);
    double GU[2][1] = { {kf->G[0][0] * iner_z}, {kf->G[1][0] * iner_z} };
    mat_add_2x1(S_pred, S_pred, GU);

    // 2. Predict the uncertainty: P(k) = F * P(k-1) * F^T + Q
    double F_T[2][2] = {{kf->F[0][0], kf->F[1][0]}, {kf->F[0][1], kf->F[1][1]}}; // Transpose of F
    mat_mult_2x2(temp1, kf->F, kf->P);
    mat_mult_2x2(P_pred, temp1, F_T);
    P_pred[0][0] += kf->Q[0][0];
    P_pred[0][1] += kf->Q[0][1];
    P_pred[1][0] += kf->Q[1][0];
    P_pred[1][1] += kf->Q[1][1];

    // 3. Compute Kalman Gain: K = P * H^T / (H * P * H^T + R)
    double H_T[2][1] = {{kf->H[0][0]}, {kf->H[0][1]}};  // Transpose of H
    // H * P
    double HP[1][2]; 
    HP[0][0] = kf->H[0][0] * P_pred[0][0] + kf->H[0][1] * P_pred[1][0]; 
    HP[0][1] = kf->H[0][0] * P_pred[0][1] + kf->H[0][1] * P_pred[1][1];
    
    // H * P * H^T + R
    double HPH_T_R = HP[0][0] * H_T[0][0] + HP[0][1] * H_T[1][0] + kf->R[0][0]; 
    
    // K_gain = P. H^T . HPH_T_R
    K[0][0] = (P_pred[0][0] * H_T[0][0] + P_pred[0][1] * H_T[1][0]) / HPH_T_R; 
    K[1][0] = (P_pred[1][0] * H_T[0][0] + P_pred[1][1] * H_T[1][0]) / HPH_T_R;

    // 4. Update the state: S(k) = S(k) + K * (M(k) - H * S(k))
    double HS = kf->H[0][0] * S_pred[0][0] + kf->H[0][1] * S_pred[1][0];
    innovation[0][0] = alt - HS;
    
    double KS[2][1] = {{K[0][0] * innovation[0][0]}, {K[1][0] * innovation[0][0]}};
    mat_add_2x1(kf->S, S_pred, KS);

    // 5. Update uncertainty: P(k) = (I - K * H) * P
    double I_KH[2][2] = {{1 - K[0][0] * kf->H[0][0],   - K[0][0] * kf->H[0][1]},
                         {  - K[1][0] * kf->H[0][0], 1 - K[1][0] * kf->H[0][1]}};
    mat_mult_2x2(kf->P, I_KH, P_pred);
}