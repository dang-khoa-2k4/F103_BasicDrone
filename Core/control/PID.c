#include "board.h"

void PID_init(PID_instance *PID, PID_instance * PID_inner, float p, float i, float d)
{
    reset_PID_gain(PID);
    PID->p_gain = p;
    PID->i_gain = i;
    PID->d_gain = d;
    if (PID_inner != NULL)
        PID->inner = *PID_inner;
    return;
}

void reset_PID_gain(PID_instance *PID)
{
    PID->p_gain = 0.0;
    PID->i_gain = 0.0;
    PID->d_gain = 0.0;
    PID->pre_error = 0;
    PID->isSaturation = false;  
    PID->integral_error = 0;
    PID->derivative_error_filter = 0;
// #ifdef INNER_PID
    set_PID_gain(&(PID->inner), NULL, 0, 0, 0);
// #endif
    return;
}


int16_t base_PID_calc(PID_instance *PID, float error_input, uint16_t sampling_rate, float filter_gain)
{
    // anti windup
    PID->integral_error += error_input * sampling_rate;  
    if (PID->isSaturation)   
        PID->integral_error = 0;

    PID->integral_error = constrain(PID->integral_error, -MAX_INTEGRAL, MAX_INTEGRAL);

    // avoid derivative kick occurs when a sudden change in the setpoint causes a spike in the derivative term of a PID controller
    float derivative_error = -(error_input - PID->pre_error) / sampling_rate;
    // low pass filter
    PID->derivative_error_filter = PID->derivative_error_filter * filter_gain + derivative_error * (1 - filter_gain);

    // PID calculation
    PID->output_PID = PID->p_gain * error_input 
                    + PID->i_gain * PID->integral_error;
                    + PID->d_gain * PID->derivative_error_filter;

    int16_t raw_value = PID->output_PID;
    PID->output_PID = constrain(PID->output_PID, -MAX_PID_VALUE, MAX_PID_VALUE);
    
    //check saturation and same sign
    PID->isSaturation = (raw_value != PID->output_PID) && (raw_value * error_input > 0); 
    PID->pre_error = error_input;

    return PID->output_PID;
}

// error angle = set_point_angle - angle
int16_t angle_PID_drone(PID_instance * PID, float error_angle, int16_t rate /* Gyro */, float filter_gain, uint16_t sampling_rate)
{
    // if(error_angle > 180.f) error_angle -= 360.f;
	// else if(error_angle < -180.f) error_angle += 360.f;
    // anti windup
    PID->integral_error += error_angle * sampling_rate;  
    if (PID->isSaturation)   
        PID->integral_error = 0;

    PID->integral_error = constrain(PID->integral_error, -MAX_INTEGRAL, MAX_INTEGRAL);

    // avoid derivative kick
    float derivative_error = - rate;
    // low pass filter
    PID->derivative_error_filter = PID->derivative_error_filter * filter_gain + derivative_error * (1 - filter_gain);

    // PID calculation
    PID->output_PID = PID->p_gain * error_angle 
                    + PID->i_gain * PID->integral_error;
                    + PID->d_gain * PID->derivative_error_filter ;

    int16_t raw_value = PID->output_PID;
    PID->output_PID = constrain(PID->output_PID, -MAX_PID_VALUE, MAX_PID_VALUE);
    
    //check saturation and same sign
    PID->isSaturation = (raw_value != PID->output_PID) && (raw_value * error_angle > 0); 
    PID->pre_error = error_angle;

    // double loop PID
    // innner loop (rate control)
// #define INNER_PID
    int16_t error_rate = PID->output_PID - rate;
    PID->output_PID = base_PID_calc(&(PID->inner), error_rate, sampling_rate, 0.5);

    return PID->output_PID;
}
