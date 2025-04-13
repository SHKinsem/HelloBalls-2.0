#ifndef __APP_MOTORS_H
#define __APP_MOTORS_H

#include "bldc_motors.h"
#include "stepper_motors.h"
#include "rm2006.h"
#include "rm3508.h"
#include "dm3519.h"

void bldc_motor_task_init(void);
void stepper_motor_task_init(void);

void home_stepper_motor(void);
void set_stepper_pos(int32_t pos);  // In steps
void set_friction_wheel_speed(float speed); // In rpm
void set_wheel_speed(float speed1, float speed2); // In rpm

#endif // __APP_MOTORS_H