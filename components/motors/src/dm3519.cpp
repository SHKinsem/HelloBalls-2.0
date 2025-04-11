#include "dm3519.h"

#ifdef USE_DM3519

std::string dm3519_error_code_to_string(DM3519_ERROR_CODE code)
{
    switch (code)
    {
        case DM3519_DISABLED:
            return "DM3519 Disabled";
        case DM3519_ENABLED:
            return "DM3519 Enabled";
        case DM3519_SENSOR_ERROR:
            return "DM3519 Sensor Error";
        case DM3519_PARAM_ERROR:
            return "DM3519 Parameter Error";
        case DM3519_OVERVOLTAGE:
            return "DM3519 Overvoltage";
        case DM3519_UNDERVOLTAGE:
            return "DM3519 Undervoltage";
        case DM3519_OVERCURRENT:
            return "DM3519 Overcurrent";
        case DM3519_MOS_OVERHEAT:
            return "DM3519 MOS Overheat";
        case DM3519_MOTOR_OVERHEAT:
            return "DM3519 Motor Overheat";
        case DM3519_LOST_COMM:
            return "DM3519 Lost Communication";
        case DM3519_OVERLOAD:
            return "DM3519 Overload";
        default:
            return "Unknown Error Code";
    }
}

twai_message_t dm3519_clear_error_msg = {
    dm3519_clear_error_msg.extd = 0,
    dm3519_clear_error_msg.rtr = 0,
    dm3519_clear_error_msg.ss = 1,
    dm3519_clear_error_msg.self = 0,
    dm3519_clear_error_msg.dlc_non_comp = 0,
    .identifier = 0,
    .data_length_code = 8,
    .data = {0},
};

#endif