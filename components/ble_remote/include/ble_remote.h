/*
 * Standalone BLE (NimBLE) GATT server for the HelloBalls remote app.
 * This component only handles Bluetooth: advertising, the connection and frame
 * decoding. It does not know about motors, the serial port or the state
 * machine; the application wires the callbacks to whatever it wants.
 *
 * GATT service b5f90001-7a4c-4e57-9c1d-48a7a3b4c0de
 *   ...0002 Control  write without response  ble_remote_cmd_t, 20-50 Hz
 *   ...0003 Action   write                   ble_remote_action_t
 *   ...0004 Status   read / notify           ble_remote_status_t, 10 Hz
 *   ...0005 Info     read                    ASCII "proto=1;od=1;fw=<version>"
 *   ...0006 SDO      write / notify          object dictionary requests and responses
 *   ...0008 TPDO     notify                  mapped object dictionary streams
 *
 * The SDO and TPDO payloads are opaque here; the obj_dict component defines them.
 * "od=1" in Info tells the app that the object dictionary characteristics exist.
 */
#ifndef BLE_REMOTE_H
#define BLE_REMOTE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "ble_remote_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

// Preferred ATT MTU; the phone may negotiate less
#define BLE_REMOTE_PREFERRED_MTU 247
#define BLE_REMOTE_MAX_PAYLOAD   (BLE_REMOTE_PREFERRED_MTU - 3)

typedef struct {
    // Called from the NimBLE host task for every valid Control frame. Keep it short.
    void (*on_command)(const ble_remote_cmd_t *cmd, void *ctx);
    // Called from the NimBLE host task for every valid Action write.
    void (*on_action)(const ble_remote_action_t *action, void *ctx);
    // Called from the NimBLE host task for every SDO write (raw bytes). Keep it short.
    void (*on_sdo)(const uint8_t *data, size_t len, void *ctx);
    // Called when a phone connects or disconnects.
    void (*on_connection)(bool connected, void *ctx);
    void *ctx;
    // Advertised name prefix; "-XXXX" (end of the BT MAC) is appended. NULL uses Kconfig.
    const char *name_prefix;
    // Reported by the Info characteristic. NULL reports "unknown".
    const char *fw_version;
} ble_remote_config_t;

// Initialise NVS (if needed), NimBLE and start advertising.
esp_err_t ble_remote_init(const ble_remote_config_t *cfg);

// Store the latest status (served on read) and notify it if the phone subscribed.
esp_err_t ble_remote_notify_status(const ble_remote_status_t *status);

// Send an SDO response / a TPDO frame if the phone subscribed. len must not exceed ble_remote_max_payload().
esp_err_t ble_remote_notify_sdo(const uint8_t *data, size_t len);
esp_err_t ble_remote_notify_tpdo(const uint8_t *data, size_t len);

// Largest notification payload for the current connection (ATT MTU - 3), at most BLE_REMOTE_MAX_PAYLOAD.
size_t ble_remote_max_payload(void);

// Manufacturer data (company ID first, little endian) for the scan response, so a
// phone can show it in the scan list without connecting. Safe to call before init.
#define BLE_REMOTE_MAX_MFG_DATA 11
esp_err_t ble_remote_set_mfg_data(const uint8_t *data, size_t len);

bool ble_remote_is_connected(void);

// Advertised device name, valid after ble_remote_init().
const char *ble_remote_device_name(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_REMOTE_H
