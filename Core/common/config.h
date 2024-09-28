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

/* Global Enums */


// void Config_Init(void);
void Config_PID(float kp, float ki, float kd);
void Config_ALL(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTORS_H__ */