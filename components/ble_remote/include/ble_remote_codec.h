/*
 * HelloBalls BLE remote protocol: fixed-size little-endian frames.
 * Pure C with no ESP-IDF dependency so it can be unit tested on a PC.
 * The Flutter app (HelloBalls-Remote, lib/ble/protocol.dart) mirrors this file;
 * both sides are checked against test/vectors.json.
 */
#ifndef BLE_REMOTE_CODEC_H
#define BLE_REMOTE_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_REMOTE_PROTOCOL_VERSION 1

#define BLE_REMOTE_CONTROL_LEN 12
#define BLE_REMOTE_ACTION_LEN  4
#define BLE_REMOTE_STATUS_LEN  14

// Control frame flags
#define BLE_REMOTE_FLAG_TAKEOVER  0x01  // App requests manual control over the host

// Action ids (Action characteristic)
#define BLE_REMOTE_ACTION_ESTOP        0x01
#define BLE_REMOTE_ACTION_CLEAR_ESTOP  0x02

// Status flags
#define BLE_REMOTE_STATUS_ESTOP     0x01
#define BLE_REMOTE_STATUS_TAKEOVER  0x02

// Active command source reported in the status frame
#define BLE_REMOTE_SRC_NONE    0
#define BLE_REMOTE_SRC_SERIAL  1
#define BLE_REMOTE_SRC_BLE     2

// Control: written by the app at 20-50 Hz, doubles as heartbeat
typedef struct {
    uint8_t seq;
    uint8_t state;        // same values as host_state_t
    uint8_t flags;        // BLE_REMOTE_FLAG_*
    int16_t wheel1_rpm;   // motor shaft RPM
    int16_t wheel2_rpm;
    int16_t tilt_angle;
    int16_t shoot_speed;
} ble_remote_cmd_t;

// Action: one-shot command written with response
typedef struct {
    uint8_t action;       // BLE_REMOTE_ACTION_*
    int16_t param;
} ble_remote_action_t;

// Status: notified by the robot at 10 Hz
typedef struct {
    uint8_t mcu_state;    // mcu_state_t
    uint8_t host_state;   // effective host_state_t
    uint8_t active_src;   // BLE_REMOTE_SRC_*
    uint8_t flags;        // BLE_REMOTE_STATUS_*
    uint8_t last_seq;     // seq of the last accepted control frame
    int16_t wheel1_rpm;   // measured
    int16_t wheel2_rpm;
    uint32_t uptime_ms;
} ble_remote_status_t;

bool   ble_remote_decode_control(const uint8_t *buf, size_t len, ble_remote_cmd_t *out);
size_t ble_remote_encode_control(const ble_remote_cmd_t *cmd, uint8_t *buf, size_t cap);

bool   ble_remote_decode_action(const uint8_t *buf, size_t len, ble_remote_action_t *out);
size_t ble_remote_encode_action(const ble_remote_action_t *act, uint8_t *buf, size_t cap);

bool   ble_remote_decode_status(const uint8_t *buf, size_t len, ble_remote_status_t *out);
size_t ble_remote_encode_status(const ble_remote_status_t *st, uint8_t *buf, size_t cap);

#ifdef __cplusplus
}
#endif

#endif // BLE_REMOTE_CODEC_H
