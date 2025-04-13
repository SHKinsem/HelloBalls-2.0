#include "dm3519.h"

#ifdef USE_DM3519

std::string dm3519_error_code_to_string(DM3519_ERROR_CODE code)
{
    switch (code)
    {
        case DM3519_DISABLED:
            return "Disabled";
        case DM3519_ENABLED:
            return "Enabled";
        case DM3519_SENSOR_ERROR:
            return "SensorErr";
        case DM3519_PARAM_ERROR:
            return "ParamErr";
        case DM3519_OVERVOLTAGE:
            return "OvrVoltage";
        case DM3519_UNDERVOLTAGE:
            return "UdrVoltage";
        case DM3519_OVERCURRENT:
            return "OvrCurr";
        case DM3519_MOS_OVERHEAT:
            return "MOSOvrheat";
        case DM3519_MOTOR_OVERHEAT:
            return "MtrOvrheat";
        case DM3519_LOST_COMM:
            return "LostCommu";
        case DM3519_OVERLOAD:
            return "Overload";
        default:
            return "Unknown";
    }
}

#endif