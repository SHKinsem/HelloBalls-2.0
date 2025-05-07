#ifndef __APP_MOTORS_H
#define __APP_MOTORS_H


#include "dm3519.h"
#include "can_channel.h"


base_motor_t* get_motor_ptr(uint8_t motor_id); // Function to get the pointer to motor

void bldc_motor_task_init(void);

// void stepper_motor_task_init(void);
// void home_stepper_motor(void);
// void set_stepper_pos(int32_t pos);  // In steps
void set_friction_wheel_speed(float speed); // In rpm
void set_wheel_speed(float speed1, float speed2); // In rpm

// Servo control functions - with C linkage for compatibility
#ifdef __cplusplus
extern "C" {
#endif

void servo_init(void); // Initialize servo on GPIO0
void set_servo_position(float angle); // Set servo position to a specific angle
bool get_servo_state(void); // Get current servo state
void toggle_servo(void); // Toggle servo position - helper function for C code

#ifdef __cplusplus
}
#endif

void twai_init();



#endif // __APP_MOTORS_H