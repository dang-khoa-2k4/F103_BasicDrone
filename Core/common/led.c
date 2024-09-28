/** @file 		led.c
 *  @brief
 *  	This file contains functions to initialize the led and control led to debug
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */

#include "board.h"

void led_0_ON(void)
{
    LL_GPIO_WritePin(led_0_GPIO_Port,  led_0_Pin, GPIO_PIN_SET);
}

void led_0_OFF(void)
{
    LL_GPIO_WritePin(led_0_GPIO_Port,  led_0_Pin, GPIO_PIN_RESET);
}

void led_0_Toggle(void)
{
    LL_GPIO_TogglePin(led_0_GPIO_Port,  led_0_Pin);
}

void led_1_ON(void)
{
    LL_GPIO_WritePin(led_1_GPIO_Port,  led_1_Pin, GPIO_PIN_SET);
}

void led_1_OFF(void)
{
    LL_GPIO_WritePin(led_1_GPIO_Port,  led_1_Pin, GPIO_PIN_RESET);
}

void led_1_Toggle(void)
{
    LL_GPIO_TogglePin(led_1_GPIO_Port,  led_1_Pin);
}

void led_2_ON(void)
{
    LL_GPIO_WritePin(led_2_GPIO_Port,  led_2_Pin, GPIO_PIN_SET);
}

void led_2_OFF(void)
{
    LL_GPIO_WritePin(led_2_GPIO_Port,  led_2_Pin, GPIO_PIN_RESET);
}

void led_2_Toggle(void)
{
    LL_GPIO_TogglePin(led_2_GPIO_Port,  led_2_Pin);
}
