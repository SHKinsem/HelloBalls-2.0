/*
 * Tunable parameters, exposed through the BLE object dictionary (0x3xxx) and
 * saved to NVS on request (0x1010). The defaults are the values that used to be
 * hard-coded in state_machine.cpp, app_motors.cpp and Kconfig.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

// Indexes into robot_params_t::pid (pid_id - 1)
#define PID_COUNT 6

typedef struct {
    // 0x3000 limits for the remote
    uint16_t max_wheel_rpm;
    uint16_t max_shoot_speed;
    int16_t tilt_min_deg;
    int16_t tilt_max_deg;

    // 0x3101-0x3106 PID {Kp, Ki, Kd, Kr, Km, Umax, Umin}: friction 1/2, wheel L/R, loader angle, loader speed
    float pid[PID_COUNT][7];

    // 0x3200 servo neutral angles
    float servo_origin[2];

    // 0x3201 searching for a ball
    int16_t search_friction_speed;
    int32_t search_stepper_pos;
    float search_tilt_down_deg;
    float search_tilt_hold_deg;
    uint16_t search_tilt_ms;

    // 0x3202 picking up a ball
    int16_t pickup_friction_speed;
    float pickup_tilt_deg;
    uint16_t pickup_tilt_ms;
    float pickup_loader1_deg;
    uint16_t pickup_loader1_ms;
    float pickup_loader2_deg;
    uint16_t pickup_loader2_ms;

    // 0x3203 shooting
    int32_t shoot_stepper_pos;
    uint16_t shoot_spinup_ms;
    float shoot_loader1_deg;
    uint16_t shoot_loader1_ms;
    float shoot_loader2_deg;
    uint16_t shoot_loader2_ms;

    // 0x2600.6 motor debug speed cap
    uint16_t debug_max_rpm;
} robot_params_t;

// Live parameters. Written only by the object dictionary task; read anywhere.
extern robot_params_t g_params;

// Load defaults, then whatever was saved. Call before anything reads g_params.
void robot_params_init(void);
esp_err_t robot_params_save(void);
// Back to the defaults in RAM (save afterwards to make it stick)
void robot_params_reset(void);
// Push the PID gains and servo origins to the hardware
void robot_params_apply(void);
