#include "app_motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// static int stepper_motor_pos = 0; // Current position of the stepper motor