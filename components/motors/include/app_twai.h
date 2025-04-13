#pragma once

#ifndef APP_TWAI_H
#define APP_TWAI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

void install_twai_driver();
void uninstall_twai_driver();
void can_task(void);
void control_motor_speed(int16_t* speed);
void twai_transmit_speed(int16_t speed);
int getGlobalSpeed();
void start_twai_receive_task(void);

#ifdef __cplusplus
}
#endif

#endif // APP_TWAI_H