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

// Last angle passed to tilt_servos() (after clamping) and whether the servos are powered
float get_tilt_angle(void);
bool servos_enabled(void);
// Change the servo neutral positions (degrees, servo frame); takes effect on the next move
void set_servo_origins(float servo1_deg, float servo2_deg);

// CAN motors by bus ID: 1/2 friction wheels, 3/4 drive wheels, 5 loader
#define APP_MOTOR_ID_FIRST 1
#define APP_MOTOR_ID_LAST  5
#define APP_MOTOR_ID_LOADER 5

typedef struct {
    int16_t speed_rpm;   // measured rotor speed, motor's own direction
    float target;        // rpm (speed loop) or degrees (loader angle loop)
    float current_a;
    float temperature_c; // -1 when the motor does not report it
    float angle_deg;     // rotor angle 0..360
    int8_t status;
    bool enabled;
} app_motor_info_t;

bool app_motor_get_info(uint8_t can_id, app_motor_info_t *out);
float app_loader_shaft_angle(void);
float app_loader_pid_error(void);

// Debug: run exactly one speed-controlled motor (IDs 1-4) at rpm in its own direction,
// every other motor disabled. can_id 0 disables all.
bool app_motor_debug_drive(uint8_t can_id, float rpm);

// PID gains {Kp, Ki, Kd, Kr, Km, Umax, Umin}. pid_id 1-5 are the motors' main loops,
// 6 is the loader's inner speed loop.
#define APP_PID_LOADER_SPEED 6
bool app_motor_set_pid(uint8_t pid_id, const float gains[7]);

#ifdef __cplusplus
}
#endif




#endif // __APP_MOTORS_H