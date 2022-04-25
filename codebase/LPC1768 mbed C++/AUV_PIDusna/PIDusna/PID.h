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

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_GAINS_TypeDef;

class PID
{
public:
    PID (float Kp, float Ki, float Kd, float dt, float deadzone=0);
    void calculate_K(float Tu);
    void set_dt(float dt);
    void clear_integral(); 
    void clear_error();
    void clear_error_previous();
    void clear();
    void set_K(float Kp, float Ki=0, float Kd=0);
    void set_deadzone(float deadzone);
    float process(float setpoint, float measured);
    float process(float error);
    void get_gain_values(PID_GAINS_TypeDef *gains);

protected:
    float _Kp;
    float _Ki;
    float _Kd;
    float _floor;
    float _deadzone;
    float _dt;
    float _error_previous;
    float _integral;

//private:

};


