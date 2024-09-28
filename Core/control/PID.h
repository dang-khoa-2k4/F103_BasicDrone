#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#define MAX_INTEGRAL 500 //
#define MAX_PID_VALUE 2000 // 


typedef struct
{
    float p_gain;
    float i_gain;
    float d_gain;
    
    PID_instance inner;

    int16_t pre_error;
    float integral_error;
    float derivative_error_filter;
    int16_t output_PID;
    bool isSaturation;
} PID_instance;

/**
  * @brief  reset all paramater in PID structure
  * @param  *PID is pointer to the PID structure
*/
void reset_PID_gain(PID_instance *PID);
/**
  * @brief  set PID paramater in PID structure
  * @param  *PID is pointer to the PID structure
  * @param  p, i, d is paramater which you want to change
*/
void set_PID_gain(PID_instance *PID, float p, float i, float d);

/**
  * @brief  set output for PID structure
  * @param  *PID is pointer to the PID structure
  * @param  error_input is error input which you need to fix it
  * @param  sampling_rate is time / 1 sampling
*/
void base_PID_calc(PID_instance *PID, int16_t error_input, uint16_t sampling_rate);

#endif 
/*PID_CONTROL_H_*/
