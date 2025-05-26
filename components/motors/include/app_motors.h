#pragma once

#ifndef __APP_MOTORS_H
#define __APP_MOTORS_H


#include "dm3519.h"
#include "can_channel.h"

#define ID_DJI_RM_MOTOR         0x200

#ifdef __cplusplus
extern "C" {
#endif


void servo_init(void); // Initialize servo on GPIO0
void toggle_servo(void); // Toggle servo position - helper function for C code
void tilt_servos(float angle); // Tilt both servos by a specified angle
void disable_servos(void); // Disable both servos
void enable_servos(void); // Enable both servos

void motor_task_init(void);
void motorStateHelper(bool state);
void set_friction_wheels_speed(const int16_t& speed); // Set speed for friction wheels
void set_wheel_motors_speed(const int16_t& speed1, const int16_t& speed2);
void moveLoader(float angle);
void resetLoaderOrigin(void);

#ifdef __cplusplus
}
#endif




#endif // __APP_MOTORS_H