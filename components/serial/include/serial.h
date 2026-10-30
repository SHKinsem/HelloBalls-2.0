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
    HOST_SCANNING,
    HOST_ERROR,
} host_state_t;

typedef enum {
    SERIAL_RECEIVING,
    SERIAL_SENDING,
    SERIAL_IDEL,
} serial_state_t;

// Define message structure for transmitting data
typedef struct {
    uint8_t mcu_state;
    uint8_t host_state; // effective host_state_t from cmd_hub
    int32_t wheel1_speed_rpm;
    int32_t wheel2_speed_rpm;
    t_sQMI8658 imu_data; // Structure containing IMU data
    uint32_t sample_sequence;
    uint64_t sample_time_us;
} tx_message_t;

// Initialize UART communication
void uart_init(void);

// Host commands are parsed line by line and submitted to cmd_hub (CMD_SRC_SERIAL)

serial_state_t* getTaskState(void);
mcu_state_t* getMcuState(void);

// Update the latest signed wheel-speed feedback sent to the host.
void set_tx_wheel_speed_feedback(int32_t wheel1_speed_rpm,
                                 int32_t wheel2_speed_rpm);

// Read back the latest signed wheel-speed feedback.
void get_tx_wheel_speed_feedback(int32_t *wheel1_speed_rpm,
                                 int32_t *wheel2_speed_rpm);

#ifdef __cplusplus
}
#endif

#endif // __SERIAL_H
