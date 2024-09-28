/** @file 		buzzer.h
 *  @brief
 *  	This file contains functions to initialize the buzzer and control buzzer to debug
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

/* Defines */

#ifdef __cplusplus
 extern "C" {
#endif

/* Global Enums */


void Buzzer_Init(void);
void Buzzer_Setting(void);
void Buzzer_Warning(void);
void Buzzer_Error(void);
void Buzzer_Success(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTORS_H__ */