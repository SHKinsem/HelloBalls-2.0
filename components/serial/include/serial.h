#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RECEIVING,
    SENDING,
    IDLE,
} task_state_t;

// Define message structure for receiving data
typedef struct {
    uint8_t machine_state;
    int16_t wheel1_speed;
    int16_t wheel2_speed;
} rx_message_t;

// Define message structure for transmitting data
typedef struct {
    uint8_t machine_state;
    int32_t wheel1_distance;
    int32_t wheel2_distance;
    float imu_x;
    float imu_y;
    float imu_z;
    float imu_yaw;
} tx_message_t;

// Initialize UART communication
void uart_init(void);

// Get a pointer to the received message data
rx_message_t* get_rx_message(void);

task_state_t* getTaskState(void);

// Set data for transmission
void set_tx_message(uint8_t state, int32_t w1_dist, int32_t w2_dist,
                    float x, float y, float z, float yaw);

#ifdef __cplusplus
}
#endif

#endif // __SERIAL_H