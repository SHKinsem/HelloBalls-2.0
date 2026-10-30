/*
 * Motor debug mode (object dictionary 0x2600): the BLE remote runs one CAN
 * motor at a chosen speed, or moves the tilt servos (0x2601), while the state machine and the command watchdog
 * leave the motors alone. Only while the remote has taken control, is fresh,
 * nothing is e-stopped and the host state is IDLE.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_DEBUG_RAMP_RPM_PER_S 1000.0f

typedef struct {
    bool mode;          // 0x2600.1 debug mode on
    uint8_t motor_id;   // 0x2600.2 CAN ID 1-4, 0 = none
    float target_rpm;   // 0x2600.3 requested speed, motor's own direction
    bool run;           // 0x2600.4 held by the app's hold-to-run button
    float applied_rpm;  // 0x2600.6 speed actually commanded after cap and ramp
    float servo_tilt;      // 0x2601.1 linked tilt: servo 1 +tilt, servo 2 -tilt (like tilt_servos)
    float servo_trim[2];   // 0x2601.2/.3 extra offset of one servo only; the two are linked, so keep small
    bool servo_touched;    // the servos were moved in this debug session
} motor_debug_t;

#define MOTOR_DEBUG_SERVO_TRIM_DEG 15.0f

extern motor_debug_t g_motor_debug;

// True while debug mode owns the motors (read by the command watchdog)
bool motor_debug_active(void);

// Object dictionary hooks: return an SDO abort code, 0 when accepted
uint32_t motor_debug_set_mode(bool on);
uint32_t motor_debug_select(uint8_t motor_id);
uint32_t motor_debug_set_run(bool run);
// sub 1 = linked tilt, 2/3 = trim of servo 1/2
uint32_t motor_debug_set_servo(uint8_t sub, float deg);
// After a servo centre changed: re-apply the debug angles around the new centre
void motor_debug_servo_origin_changed(void);

// Leave debug mode and stop every motor
void motor_debug_exit(const char *why);

// Called every dt_ms by the object dictionary task: checks the conditions, ramps and drives
void motor_debug_tick(uint32_t dt_ms);
