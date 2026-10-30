/*
 * Boot stage and fault reasons, so the phone can tell why the robot is not
 * ready: in the BLE advertisement (seen from the scan list, no connection
 * needed) and in the object dictionary (0x2001).
 *
 * Manufacturer data in the scan response, 10 bytes:
 *   company 0xFFFF (u16 LE) | 'H' 'B' | version 1 | stage u8 | faults u32 LE
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HEALTH_STAGE_BOOT = 0,
    HEALTH_STAGE_BLE,
    HEALTH_STAGE_IMU,
    HEALTH_STAGE_CAN,
    HEALTH_STAGE_SERVO,
    HEALTH_STAGE_STEPPER,
    HEALTH_STAGE_HOMING,
    HEALTH_STAGE_SERIAL,
    HEALTH_STAGE_READY,
} health_stage_t;

// Fault bits (0x2001.2). Keep in step with the app (lib/ble/health.dart).
#define HEALTH_F_IMU            (1u << 0)  // IMU did not answer during init
#define HEALTH_F_CAN            (1u << 1)  // CAN driver did not start
#define HEALTH_F_MOTOR(id)      (1u << (1 + (id)))  // CAN ID 1-5 sends no feedback: bits 2-6
#define HEALTH_F_HOMING         (1u << 7)  // stepper homing timed out, endstop never triggered
#define HEALTH_F_SERIAL         (1u << 8)  // no commands from the serial host
#define HEALTH_F_ESTOP          (1u << 9)  // emergency stop latched

#define HEALTH_MFG_LEN 10

// The step setup() is working on; also stamps when it started
void health_set_stage(health_stage_t stage);
health_stage_t health_stage(void);
uint32_t health_faults(void);

// Re-evaluate the faults from the hardware; call every ~100 ms. Updates the advertisement on change.
void health_refresh(void);

// "卡在回零；左轮（CAN 3）无反馈" style text for the current stage and faults
void health_reason(char *buf, size_t cap);

#ifdef __cplusplus
}
#endif
