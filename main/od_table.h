/*
 * The robot's object dictionary: every field the BLE app can read or write.
 * Index map (see the design doc):
 *   0x1xxx device, descriptor, TPDO settings   0x2000 robot status
 *   0x2201-0x2205 CAN motors by bus ID          0x2300 servos, 0x2400 stepper, 0x2500 IMU
 *   0x2600 motor debug mode                     0x3xxx parameters (0x1010 saves them)
 *   0x5000 emergency stop
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "obj_dict.h"

#define OD_TPDO_COUNT 4

typedef struct {
    uint8_t count;                  // 0x1A0n.0 number of mapped entries
    uint32_t map[OD_PDO_MAX_MAP];   // 0x1A0n.1-8
    uint16_t period_ms;             // 0x180n.1
    bool enabled;                   // 0x180n.2
    uint8_t seq;
    int64_t last_us;
} od_tpdo_t;

extern od_tpdo_t g_tpdo[OD_TPDO_COUNT];
extern const od_dict_t g_od;

// Check the table and build the descriptor. Call once before serving requests.
void od_table_init(void);

// Copy the live values (motors, IMU, status...) the entries point at. Call from the
// object dictionary task before answering requests or packing TPDOs.
void od_table_refresh(void);

// Turn every TPDO off and restore the default mappings (new connection)
void od_table_reset_tpdo(void);
