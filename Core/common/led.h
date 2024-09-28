/** @file 		led.h
 *  @brief
 *  	This file contains functions to initialize the led and control led to debug
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */

#ifndef __LED_H__
#define __LED_H__

/* Defines */
#ifdef __cplusplus
 extern "C" {
#endif

/* Function Prototypes */
void led_0_ON(void);
void led_0_OFF(void);
void led_0_Toggle(void);

void led_1_ON(void);
void led_1_OFF(void);
void led_1_Toggle(void);

void led_2_ON(void);
void led_2_OFF(void);
void led_2_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H__ */