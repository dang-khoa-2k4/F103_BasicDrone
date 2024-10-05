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
void led_arm_ON(void);
void led_arm_OFF(void);
void led_arm_Toggle(void);

void led_battery_ON(void);
void led_battery_OFF(void);
void led_battery_Toggle(void);

void led_optional_ON(void);
void led_optional_OFF(void);
void led_optional_Toggle(void);

void leds_Set(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H__ */