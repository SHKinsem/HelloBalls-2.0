#pragma once

#ifndef APP_TWAI_H
#define APP_TWAI_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TWAI_TX_PIN
    #define TWAI_TX_PIN             11
#endif

#ifndef TWAI_RX_PIN
    #define TWAI_RX_PIN             10
#endif

#include <stdint.h>
#include <stdbool.h>

#define ID_DJI_RM_MOTOR         0x200

void install_twai_driver();
void uninstall_twai_driver();
void can_task(void);
void control_motor_speed(int16_t* speed);
void twai_transmit_speed(int16_t speed1, int16_t speed2);
// int getGlobalSpeed();
uint8_t* motorDataHook(uint8_t motor_id);
void start_twai_receive_task(void);
void start_twai_receive();
void stop_twai_receive();
bool get_twai_running();


#ifdef __cplusplus
}
#endif

#endif // APP_TWAI_H