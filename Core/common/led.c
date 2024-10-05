/** @file 		led.c
 *  @brief
 *  	This file contains functions to initialize the led and control led to debug
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */

#include "board.h"

void led_arm_ON(void)
{
    LL_GPIO_WritePin(led_0_GPIO_Port,  led_0_Pin, GPIO_PIN_SET);
}

void led_arm_OFF(void)
{
    LL_GPIO_WritePin(led_0_GPIO_Port,  led_0_Pin, GPIO_PIN_RESET);
}

void led_arm_Toggle(void)
{
    LL_GPIO_TogglePin(led_0_GPIO_Port,  led_0_Pin);
}

void led_battery_ON(void)
{
    LL_GPIO_WritePin(led_1_GPIO_Port,  led_1_Pin, GPIO_PIN_SET);
}

void led_battery_OFF(void)
{
    LL_GPIO_WritePin(led_1_GPIO_Port,  led_1_Pin, GPIO_PIN_RESET);
}

void led_battery_Toggle(void)
{
    LL_GPIO_TogglePin(led_1_GPIO_Port,  led_1_Pin);
}

void led_optional_ON(void)
{
    LL_GPIO_WritePin(led_2_GPIO_Port,  led_2_Pin, GPIO_PIN_SET);
}

void led_optional_OFF(void)
{
    LL_GPIO_WritePin(led_2_GPIO_Port,  led_2_Pin, GPIO_PIN_RESET);
}

void led_optional_Toggle(void)
{
    LL_GPIO_TogglePin(led_2_GPIO_Port,  led_2_Pin);
}

void leds_Set(void)
{
    armed ? led_arm_ON() : led_arm_OFF();
    
    if (config_done)
        battLow     ?   led_battery_ON()    : 
        battEmpty   ?   led_battery_OFF()   : 
                        led_battery_Toggle();
}
