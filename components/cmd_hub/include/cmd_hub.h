/*
 * Command hub: every command source (serial host, BLE remote) submits commands
 * here, and the state machine reads only from here. The hub decides which
 * source is in control, stops the robot when the active source goes quiet,
 * and latches the emergency stop.
 */
#ifndef CMD_HUB_H
#define CMD_HUB_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CMD_SRC_SERIAL = 0,
    CMD_SRC_BLE,
    CMD_SRC_COUNT,
} cmd_source_t;

// Same fields as the serial command from the host
typedef struct {
    uint8_t host_state;   // host_state_t
    int16_t wheel1_speed; // motor shaft RPM
    int16_t wheel2_speed;
    int16_t tilt_angle;
    int16_t shoot_speed;
} robot_cmd_t;

typedef struct {
    cmd_source_t active;  // source whose commands are executed
    bool takeover;        // BLE has taken control from the serial host
    bool estop;           // emergency stop latched
    bool fresh[CMD_SRC_COUNT];
    uint8_t host_state;   // effective host state
} cmd_hub_status_t;

// A source is stale when it has not sent a command for this long
#define CMD_HUB_SOURCE_TIMEOUT_MS 300

void cmd_hub_init(void);

// Submit a command from a source. Ignored unless that source is in control.
void cmd_hub_submit(cmd_source_t src, const robot_cmd_t *cmd);

// BLE asks for (or releases) control. Any change of controller forces HOST_IDLE first.
void cmd_hub_request_takeover(bool takeover);

// A source disconnected: if it was in control, stop and hand control back.
void cmd_hub_source_lost(cmd_source_t src);

// Latch or clear the emergency stop. Latching forces HOST_IDLE.
void cmd_hub_set_estop(bool estop);

// Wait for the next accepted command. Returns false on timeout or while e-stopped.
bool cmd_hub_wait_command(robot_cmd_t *out, TickType_t timeout);

// Wait until the effective host state changes, then return it.
uint8_t cmd_hub_wait_state_change(TickType_t timeout, bool *changed);

uint8_t cmd_hub_host_state(void);
void cmd_hub_get_status(cmd_hub_status_t *out);

#ifdef __cplusplus
}
#endif

#endif // CMD_HUB_H
