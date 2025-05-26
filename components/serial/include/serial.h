#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>
#include "esp32_s3_szp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MCU_IDLE = 0,
    MCU_SEARCHING_BALL,
    MCU_BALL_RETRIEVED,
    MCU_SHOOTING,
    MCU_SHOOTING_COMPLETE,
    MCU_ERROR,
} mcu_state_t;

typedef enum {
    HOST_IDLE = 0,
    HOST_SEARCHING_BALL,
    HOST_BALL_REACHED,
    HOST_SHOOTING,
    HOST_ERROR,
} host_state_t;

typedef enum {
    SERIAL_RECEIVING,
    SERIAL_SENDING,
    SERIAL_IDEL,
} serial_state_t;

// Define message structure for receiving data
typedef struct {
    uint8_t host_state;
    int16_t wheel1_speed;
    int16_t wheel2_speed;
    int16_t tilt_angle;
} rx_message_t;

// Define message structure for transmitting data
typedef struct {
    uint8_t mcu_state;
    uint8_t host_state; // 0: SERIAL_IDEL, 1: SEARCHING_BALL, 2: SHOOTING
    int32_t wheel1_distance;
    int32_t wheel2_distance;
    t_sQMI8658 imu_data; // Structure containing IMU data
} tx_message_t;

// Initialize UART communication
void uart_init(void);

// Get a pointer to the received message data
rx_message_t* get_rx_message_ptr(void);
rx_message_t get_rx_message(void);

serial_state_t* getTaskState(void);
host_state_t* getHostState(void);
mcu_state_t* getMcuState(void);

// Set data for transmission
void set_tx_message(uint8_t state, int32_t w1_dist, int32_t w2_dist,
                    float x, float y, float z, float yaw);

#ifdef __cplusplus
}
#endif

#endif // __SERIAL_H