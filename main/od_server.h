/*
 * Serves the object dictionary over the BLE remote: SDO requests are queued
 * from the NimBLE callback and answered by one task, which also sends the
 * enabled TPDOs and runs the motor debug mode. Everything that touches the
 * dictionary happens in that task.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void od_server_init(void);

// ble_remote callbacks (NimBLE host task)
void od_server_on_sdo(const uint8_t *data, size_t len);
void od_server_on_connection(bool connected);
