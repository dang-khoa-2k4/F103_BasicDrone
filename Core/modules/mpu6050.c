/*
 * mpu6050.c
 *
 *  Created on: Jun 30, 2024
 *      Author: Nguyen Tran Dang Khoa
 */

#include "mpu6050.h"

uint8_t mpu6050_init(MPU_6050 * mpu)
{
	uint8_t data;
	uint8_t mpu_check;

	HAL_I2C_Mem_Read(MPU6050_I2C, MPU6050_ADDR, WHO_AM_I, 1, &mpu_check, 1, i2c_timeout);

	if (mpu_check == 0x68) // default value
	{
		// Power up the device
		data = 0x00;
		HAL_I2C_Mem_Write(MPU6050_I2C, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, i2c_timeout);
		// Turn on Low pass filter
		data = 0x05;
		HAL_I2C_Mem_Write(MPU6050_I2C, MPU6050_ADDR, CONFIG, 1, &data, 1, i2c_timeout);
		// Set up Gyro's Full scale rate +- 500 */s
		data = 0x08;
		HAL_I2C_Mem_Write(MPU6050_I2C, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, i2c_timeout);
		// Set up Acceler's Full scale rate +- 8g
		data = 0x10;
		HAL_I2C_Mem_Write(MPU6050_I2C, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, i2c_timeout);
		// Set up sampling rate to 1kHz
		data = 0x00;
		HAL_I2C_Mem_Write(MPU6050_I2C, MPU6050_ADDR, SMPLRT_DIV, 1, &data, 1, i2c_timeout);
		// get reference point
		mpu->ref_point_X = 0;
		mpu->ref_point_Y = 0;
		mpu->ref_point_Z = 0;
		get_ref_point(mpu);
		// If init success return 1
		return 1;
	}
	// Else return 0
	return 0;
}

void get_ref_point(MPU_6050 * mpu)
{
	int32_t ref_x = 0;
	int32_t ref_y = 0;
	int32_t ref_z = 0;
	for (int i = 0; i < TIME_REF; i++)
	{
		mpu6050_update_all(mpu);
		ref_x += mpu->rateRoll;
		ref_y += mpu->ratePitch;
		ref_z += mpu->rateYaw;
		HAL_Delay(1); // sampling time is 1kHz (1 ms)
	}
	mpu->ref_point_X = ref_x / TIME_REF;
	mpu->ref_point_Y = ref_y / TIME_REF;
	mpu->ref_point_Z = ref_z / TIME_REF;
}

void concat_raw_data(MPU_6050 * mpu, uint8_t * half_data)
{
	mpu->X_raw = ((int16_t)(half_data[0] << 8)) | half_data[1];
	mpu->Y_raw = ((int16_t)(half_data[2] << 8)) | half_data[3];
	mpu->Z_raw = ((int16_t)(half_data[4] << 8)) | half_data[5];
}

void mpu6050_update_all(MPU_6050 * mpu)
{
	uint8_t half_data[6];
	// read raw value of x,y,z gyro into half_data
	HAL_I2C_Mem_Read(MPU6050_I2C, MPU6050_ADDR, GYRO_XOUT_H, 1, half_data, 6, i2c_timeout);
	// concat data
	concat_raw_data(mpu, half_data);
	// convert to physical data and calibrate
	mpu->rateRoll  = ((double) mpu->X_raw) / PHYSICAL_CONVERT_GYRO - mpu->ref_point_X;
	mpu->ratePitch = ((double) mpu->Y_raw) / PHYSICAL_CONVERT_GYRO - mpu->ref_point_Y;
	mpu->rateYaw   = ((double) mpu->Z_raw) / PHYSICAL_CONVERT_GYRO - mpu->ref_point_Z;
	// got ref point -> valid rate
	if (mpu->ref_point_X)
	{
		// read raw value of x,y,z accel into half_data
		HAL_I2C_Mem_Read(MPU6050_I2C, MPU6050_ADDR, ACCEL_XOUT_H, 1, half_data, 6, i2c_timeout);
		// concat data
		concat_raw_data(mpu, half_data);
		// convert to physical data
		mpu->Ax = ((double) mpu->X_raw) / PHYSICAL_CONVERT_ACCEL - CALIB_Ax_VALUE;
		mpu->Ay = ((double) mpu->Y_raw) / PHYSICAL_CONVERT_ACCEL - CALIB_Ay_VALUE;
		mpu->Az = ((double) mpu->Z_raw) / PHYSICAL_CONVERT_ACCEL - CALIB_Az_VALUE;
	}
}

