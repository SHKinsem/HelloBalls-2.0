#include "app_motors.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include <memory.h>

#define TAG "motors"

base_motor_t::base_motor_t(uint8_t motor_id){
    this->motor_id = motor_id;
    this->raw_speed = 0;
    this->raw_current = 0;
    this->raw_angle = 0;
    this->raw_status = 0;
    this->target_speed = 0;
    this->target_angle = 0;
    this->temperature = 0;

    // Initialize PID parameters
    speed_pid.param.Kp = _IQ(10.0);      // Proportional gain
    speed_pid.param.Ki = _IQ(0);        // Integral gain
    speed_pid.param.Kd = _IQ(0);        // Derivative gain
    speed_pid.param.Kr = _IQ(1.0);      // Reference weight, usually set to 1
    speed_pid.param.Km = _IQ(0);      // Derivative weight
    speed_pid.param.Umax = _IQ(500.0);   // Output upper limit
    speed_pid.param.Umin = _IQ(-500.0);  // Output lower limit

    // Set up differential filter coefficients
    speed_pid.term.c1 = _IQ(0);     // Differential filter coefficient 1
    speed_pid.term.c2 = _IQ(0);    // Differential filter coefficient 2
}

base_motor_t::~base_motor_t(){
    // Destructor implementation if needed
}

int16_t base_motor_t::calOutput(){

    // Set PID input values
    speed_pid.term.Ref = _IQ(this->target_speed);     // Set reference input (target speed)
    speed_pid.term.Fbk = _IQ(this->raw_speed);        // Set feedback value (current speed)

    // Execute PID calculation
    PID_MACRO(speed_pid);

    // Get control output
    int16_t control_output = _IQint(speed_pid.term.Out);    
    // int16_t control_output = speed_pid.term.Out;

    if(!enabled) return 0; // If motor is not enabled, return 0

    return control_output;
}
