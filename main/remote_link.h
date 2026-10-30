#ifndef REMOTE_LINK_H
#define REMOTE_LINK_H

// Wires the standalone ble_remote component to cmd_hub. No-op when the BLE remote is disabled.
void remote_link_init(void);

// Call once motors, servos and the stepper are initialised. Until then a BLE
// e-stop only latches in cmd_hub and does not touch the motor drivers.
void remote_link_set_hw_ready(void);

// Latch the emergency stop and cut every actuator now
void remote_link_emergency_stop(void);

#endif // REMOTE_LINK_H
