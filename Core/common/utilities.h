/** @file 		utilities.h
 *  @brief
 *  	This file contains random useful functions.
 *
 *  @author 	Khoa Nguyen
 *  @date 		28 SAT 2024
 */


#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include "stdint.h"
/* Defines */
#define PI 3.14159265358979323846

/* Defines */
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
#define FREQ_TO_TICK_CONTROL(x) (TIM_FREQ / x)

/* Function Prototypes */
/** @brief Constrains an input between two values.
 *
 *  @param input Value to be constrained.
 *  @param minValue Lower threshold.
 *  @param maxValue Upper threshold.
 *  @return float Constrained value.
 */
float constrain(float input, float minValue, float maxValue);

/** @brief Constrains an input uint16_t between two values.
 *
 *  @param input Value to be constrained.
 *  @param minValue Lower threshold.
 *  @param maxValue Upper threshold.
 *  @return uint16_t Constrained value.
 */
uint16_t constrain16(uint16_t input, uint16_t minValue, uint16_t maxValue);

/** @brief Puts a value in standard radian format.
 *
 *  @param angle Value to be formatted.
 *  @return float Formatted value.
 */
float standardRadianFormat(float angle);


/** @brief Simple and fast atof (ascii to float) function.
 *  @param *p Pointer to string.
 *  @return float Converted float value.
 */
float stringToFloat(const char *p);


#endif /* __UTILITIES_H__ */
