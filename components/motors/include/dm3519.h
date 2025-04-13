#ifndef DM3519_H
#define DM3519_H

#include "app_motors.h"

#ifdef USE_DM3519

#include <string>
#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

enum DM3519_ERROR_CODE
{
    DM3519_DISABLED = 0x00,
    DM3519_ENABLED = 0x01,
    DM3519_SENSOR_ERROR = 0x05,
    DM3519_PARAM_ERROR = 0x06,
    DM3519_OVERVOLTAGE = 0x08,
    DM3519_UNDERVOLTAGE = 0x09,
    DM3519_OVERCURRENT = 0x0A,
    DM3519_MOS_OVERHEAT = 0x0B,
    DM3519_MOTOR_OVERHEAT = 0x0C,
    DM3519_LOST_COMM = 0x0D,
    DM3519_OVERLOAD = 0x0E,
};

std::string dm3519_error_code_to_string(DM3519_ERROR_CODE code);
    
twai_message_t dm3519_clear_error_msg;

#ifdef __cplusplus
}
#endif


#endif

#endif // __cplusplus
