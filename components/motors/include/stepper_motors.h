#ifndef __STEPPER_MOTORS_H
#define __STEPPER_MOTORS_H

#include "../../main/board_pins.h"

#ifndef DIR_PIN
  #define DIR_PIN 11
#endif

#ifndef STEP_PIN
  #define STEP_PIN 10
#endif

#ifndef ENDSTOP_PIN
  #define ENDSTOP_PIN 18 // Default endstop switch pin
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize stepper engine and attach stepper to pins
void stepper_motor_task_init(void);
// Perform homing using endstop switch, resets position to 0.
// Returns false if the endstop did not trigger within the timeout.
bool home_stepper_motor(void);
// True once homing has succeeded; moves are refused until then
bool stepper_is_homed(void);
// Move stepper to specified absolute position (ignored while not homed)
void set_stepper_pos(int32_t pos);
// Stop any move immediately (emergency stop)
void stop_stepper_motor(void);

#ifdef __cplusplus
}
#endif

#endif // __STEPPER_MOTORS_H