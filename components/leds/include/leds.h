#pragma once

#include "../../main/board_pins.h"
#include "../../../main/HelloBalls.h"

#ifndef __LEDS_H
#define __LEDS_H

#ifdef __cplusplus
extern "C" {
#endif



void led_example();
void led_init(void);
void update_led_state_noHandle(State_t state);

#ifdef __cplusplus
}
#endif

#endif // __LEDS_H