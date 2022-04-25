/*
 * PID class for mbed
 * US Naval Academy
 * Robotics and Control TSD
 * Patrick McCorkell
 *
 * Created: 2019 Dec 11
 * 
 */

#include "mbed.h"
#include "PID.h"

// K values as floats.
// dt should be set to same as Ticker that uses this class.
// deadzone represents the acceptable error, defaults to 0.
PID::PID (float Kp, float Ki, float Kd, float dt, float deadzone)
{
    set_dt(dt);
    set_K(Kp, Ki, Kd);
    set_deadzone(deadzone);
    clear();
}

/*
Notes from Levi DeVries, PhD:
The method goes like this:
1) Set Ti and Td to zero. Turn Kp up until the response is borderline unstable (it oscillates without settling to a constant). Write down that number, call it Ku. Also, note the period of the oscillations in the marginally stable output, call that period Tu.
2) I assume you are going for a response with little to no overshoot. To achieve this choose, Kp = Ku/5, Ti = Tu/2, and Td = Tu/3 (if you are choosing gains by time constants). If you choosing gains instead of time constants Ki = (2/5)*Ku/Tu, Kd = Ku*Tu/15.
*/
void PID::calculate_K(float Tu)
{
    float calc_Kp = _Kp/5;
    float calc_Ki = (2*calc_Kp)/Tu;
    float calc_Kd = (calc_Kp * Tu)/3;
    set_K(calc_Kp,calc_Ki,calc_Kd);
}

// PID::PID_calculate_K(float max_error, float output_range, float P_max, float timeframe, float time_factor, int dt, float deadzone)
// {
    // //Initialize dt and deadzone values.
    // set_dt(dt);
    // set_deadzone(deadzone);
    
    // //Calculate the proportion to reach Max P value at Max Error value.
    // float Kp=(P_max/max_error);
    
    // //Calculate the integral to reach Max range at (factor * target) time (ie if target is 5s and factor is 2, Integral will drive to reach max PWM at 10s).
    // float iterations=time_factor*(timeframe/_dt);
    // float error_delta=(max_error/iterations);
    // float integral = 0;
    // float error=max_error;
    // //Find the total integral at (factor * target) time in the max distance scenario.
    // int i=0;
    // while (i<iterations) {
        // integral+=error;
        // //error starts off at max, and slowly declines to 0 as approaching target.
        // error-=error_delta;
        // i++;
    // }
    // float Ki=(output_range - P_max)/integral;
    
    // //Calculate Kd to cancel out Ki if we're on schedule to reach target under timeframe.
    // //Start by rerunning error from Max to 0, but within the specified timeframe.
    // iterations=(timeframe/_dt);
    // error_delta=(max_error/iterations);
    // integral=0;
    // error=max_error;
    // i=0;
    // while (i<iterations) {
        // integral+=error;
        // error-=error_delta;
        // i++;
    // }
    // //Having reran the integral, calculate the integral gain to offset.
    // float gainI=Ki*integral;
// }

// Change dt.
void PID::set_dt(float dt) 
{
    _dt=dt;
}

// For instances when the Integral should be zeroed out.
void PID::clear_integral() 
{
    _integral=0;
}

// For instances when the last error should be zeroed out.
void PID::clear_error_previous() 
{
    _error_previous=0;
}

void PID::clear()
{
    clear_integral();
    clear_error_previous();
}

// Sets K values for all 3 terms.
void PID::set_K(float Kp, float Ki, float Kd) 
{
    // Setup proportional value.
    _Kp=Kp;
    
    // Setup integral values.
    clear_integral();
    _Ki=Ki;
    
    // Setup derivative values.
    clear_error_previous();
    _Kd=Kd;
}

// Sets deadzone, if applicable.
void PID::set_deadzone(float deadzone) 
{
    _deadzone=deadzone;
}

// Calculate the PID from setpoint and measured.
// Returns the gain.
float PID::process(float setpoint, float measured) 
{
    // Calculate the error.
    float error=setpoint-measured;
    
    // If abs value of error is smaller than the deadzone,
    //  cause all the PID gains to zeroize.
    if (abs(error)<_deadzone) {
        clear_integral();
        clear_error_previous();
        error=0;
    }
    
    // Proportional = Kp * e
    float k_term = (_Kp*error);
    
    // Integral = Ki * e dt
    _integral+=(error*_dt);
    float i_term = (_Ki*_integral);
    
    // Derivative = Kd * (de/dt)
    float d_term = (_Kd* ((error-_error_previous)/_dt) );
    
    // PID = P + I + D
    float PID_calc = k_term+i_term+d_term;

    // Update last error for next Derivative calculation.
    _error_previous=error;
    
    // Return the calculated PID gain.
    return PID_calc;
}

// Overloaded version to give function the error directly.
float PID::process(float error)
{
    float k_term=0;
    float i_term=0;
    float d_term=0;
    float out_PID=0;
    // If abs value of error is smaller than the deadzone,
    //  cause all the PID gains to zeroize.
    if ((abs(error))<_deadzone) {
        clear_integral();
        clear_error_previous();
        error=0;
    }

    // Proportional = Kp * e
    k_term = (_Kp*error);

    // Integral = Ki * e dt
    _integral+=(error*_dt);
    i_term = (_Ki*_integral);

    // Derivative = Kd * (de/dt)
    d_term = (_Kd* ((error-_error_previous)/_dt) );

    // PID = P + I + D
    out_PID = k_term+i_term+d_term;

    // Update last error for next Derivative calculation.
    _error_previous=error;
    return out_PID;
}

void PID::get_gain_values(PID_GAINS_TypeDef *gains) {
    gains->Kp = _Kp;
    gains->Ki = _Ki;
    gains->Kd = _Kd;
}

