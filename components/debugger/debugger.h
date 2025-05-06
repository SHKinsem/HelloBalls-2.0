#ifndef __DEBUGGER
#define __DEBUGGER

#include "esp32_s3_szp.h"

void debugLoggingTask(void *arg);
void serialWheelControlTask(void *arg);
void motor_task_init(void);
void measure_important_function(void);

#endif