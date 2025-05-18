#ifndef __APP_MOTORS_H
#define __APP_MOTORS_H


#include "dm3519.h"
#include "can_channel.h"

#define ID_DJI_RM_MOTOR         0x200

// Servo control parameters - use different GPIO and timer/channel combination
#define SERVO_GPIO              SERVO_PIN         // Changed from GPIO0 to GPIO18
#define LEDC_TIMER              LEDC_TIMER_1  // Changed from TIMER_0 to TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_1  // Changed from CHANNEL_0 to CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          50                // PWM frequency in Hz (standard for servo is 50Hz)
#define SERVO_MIN_PULSEWIDTH    500               // Minimum pulse width in microseconds (0 degrees)
#define SERVO_MAX_PULSEWIDTH    2500              // Maximum pulse width in microseconds (180 degrees)
#define SERVO_ANGLE_0           0                 // 0 degree position
#define SERVO_ANGLE_30          30                // 30 degree position

#ifdef __cplusplus
extern "C" {
#endif

void servo_init(void); // Initialize servo on GPIO0
void set_servo_position(float angle); // Set servo position to a specific angle
bool get_servo_state(void); // Get current servo state
void toggle_servo(void); // Toggle servo position - helper function for C code
void serialWheelControlTask(void *arg);
void motor_task_init(void);

#ifdef __cplusplus
}
#endif




#endif // __APP_MOTORS_H