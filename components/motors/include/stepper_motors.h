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

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize stepper engine and attach stepper to pins
void stepper_motor_task_init(void);
// Perform homing using endstop switch, resets position to 0
void home_stepper_motor(void);
// Move stepper to specified absolute position
void set_stepper_pos(int32_t pos);

#ifdef __cplusplus
}
#endif

#endif // __STEPPER_MOTORS_H