/** @file 		battery.h
 *  @brief
 *  	This file consist all function about battery.
 *
 *  @author 	Khoa Nguyen
 *  @date 		20 SEP 2024
 */
#ifndef SRC_BATTERY_H_
#define SRC_BATTERY_H_

#include "stdint.h"
#include "board.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Enumerations */
typedef enum
{
	NO_BATT,
	THREE = 3,
	FOUR,
	FIVE,
	SIX
}battCells_e;

/* Global Variables */
extern float battVoltage;
extern battCells_e cells;
extern bool battLow;
extern bool battEmpty;

/* Function Prototypes */

/** 
 *  @brief Initializes the battery monitor.
 *  @param None.
 *  @return Void.
 */
void battMonInit(void);

/** 
 *  @brief Starts the ADC conversion and processes the value.
 *  @param None.
 *  @return Void.
 */
void battMonRead(void);

#ifdef __cplusplus
}
#endif


#endif /* SRC_BATTERY_H_ */