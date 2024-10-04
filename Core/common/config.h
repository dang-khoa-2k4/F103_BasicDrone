/** @file 		config.h
 *  @brief
 *  	This file configures all settings of the flight controller.
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
 extern "C" {
#endif

#define kP_roll 10.0
#define kI_roll 10.0
#define kD_roll 10.0

#define kP_pitch 10.0
#define kI_pitch 10.0
#define kD_pitch 10.0

#define kP_yaw 10.0
#define kI_yaw 10.0
#define kD_yaw 10.0

#define kP_inner_roll 10.0
#define kI_inner_roll 10.0
#define kD_inner_roll 10.0

#define kP_inner_pitch 10.0
#define kI_inner_pitch 10.0
#define kD_inner_pitch 10.0

// use for converting the raw values from the receiver to the actual values
#define ANGLE_SCALING(X) (15.0 * (X)/ (1 << 11))
#define THROTTLE_SCALING(X) ((125.0f / 1024) * (X) + 1500)
 
/* Global Enums */
extern PID_instance pid_roll;
extern PID_instance pid_pitch;
extern PID_instance pid_yaw;
extern PID_instance pid_inner_roll;
extern PID_instance pid_inner_pitch;

void Config_Init(void);
// void Config_PID(float kp, float ki, float kd);
// void Config_ALL(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTORS_H__ */