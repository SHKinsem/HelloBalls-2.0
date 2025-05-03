#ifndef __APP_MOTORS_H
#define __APP_MOTORS_H


#include "dm3519.h"


// extern m3508_t motor1; // Create an instance of base_motor_t for motor 1

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
void servo_rotate(bool direction); // Rotate servo (true = 30 degrees, false = 0 degrees)
bool get_servo_state(void); // Get current servo state
void toggle_servo(void); // Toggle servo position - helper function for C code

#ifdef __cplusplus
}
#endif

void twai_init();

// class can_channel_t
// {
// private:
//     uint8_t channel_id; // Channel ID for the CAN channel
//     uint8_t motorCount;
//     base_motor_t** motors;
//     uint8_t** rx_buffer;
//     uint8_t* tx_buffer;

// public:
//     can_channel_t(uint8_t channel_id) : channel_id(channel_id) {}
//     ~can_channel_t() {}

//     void reg_motor(base_motor_t* motor) {
//         if (motorCount < 4) { // Assuming a maximum of 4 motors per channel
//             motors[motorCount++] = motor;
//             rx_buffer[motorCount - 1] = new uint8_t[8]; // Allocate memory for the motor's RX buffer
//         } else {
//             // Handle error: too many motors registered
//         }
//     }
//     void setChannelId(uint8_t channel_id) { this->channel_id = channel_id; }
//     uint8_t getChannelId() const { return this->channel_id; }
// };

#endif // __APP_MOTORS_H