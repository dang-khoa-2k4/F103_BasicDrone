/** @file 		battery.h
 *  @brief
 *  	This file consist all function about battery.
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */

#include "battery.h"
 
/* Global Functions */
float battVoltage;
battCells_e cells;
uint8_t battLow = 0;
uint8_t battEmpty = 0;

static battCells_e getBattCells() {
	float summedVoltage = 0;
	float avgVoltage = 0;
	HAL_Delay(10);
	for(uint8_t i = 0; i < 10; i++)
	{
		battMonRead();
		summedVoltage += battVoltage;
		delay(10);
	}
	avgVoltage = summedVoltage / 10.0f;

	if((avgVoltage >= 10.0f) && (avgVoltage < 13.3))
		return THREE;
	else if((avgVoltage >= 13.3) && (avgVoltage < 17.15))
		return FOUR;
	else if((avgVoltage >= 17.15) && (avgVoltage < 21))
		return FIVE;
	else if(avgVoltage >= 21)
		return SIX;
	else
		return NO_BATT;
}

void battMonInit(void)
{
    HAL_Delay(1000);
    cells = getBattCells();
    if (cells) printf("Battery cells: %d\n", cells);
    else printf("No battery detected\n");
}

void battMonRead(void)
{
    HAL_ADC_Start(BATT_ADC);
    HAL_ADC_PollForConversion(BATT_ADC, 100);
    battVoltage = (HAL_ADC_GetValue(BATT_ADC) * 3.3) / 4096;
    HAL_ADC_Stop(BATT_ADC);
    checkBattery();
}

static void checkBattery(void)
{
    if(cells != 0)
	{
		if(battVoltage < (cells * 3.4)){
			// color(RED, YES);
			printf("\nBATTERY EMPTY\n");
			colorDefault();
			battEmpty = 1;
		}
		else if(battVoltage < (cells * 3.7)){
			// color(YELLOW, YES);
			printf("\nBATTERY LOW\n");
			colorDefault();
			battLow = 1;
		}
		else
		{
			battLow = 0;
			battEmpty = 0;
		}
	}
}

