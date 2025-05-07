#include "dm3519.h"

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

char* dm3519_t::getMotorInfo(){
    snprintf(motor_info, sizeof(motor_info),
        "Motor ID: %1u:\n"
        "Angle:\n\t%3.2f Degrees\n"      // Fixed width for angle
        "Speed:\n\t%5d RPM\n"            // Fixed width for RPM
        "TargetSpeed:\n\t%5d RPM\n"     // Fixed width for target speed
        "Current:\n\t%2.2f Amps\n"       // Fixed width for current
        "Temperature:\n\t%3.2f Celsius\n"// Fixed width for temp
        "Status:\t%s",                   // Status string
        this->getMotorId(),
        this->getAngle(),          // e.g. " 360.0" (always 6 characters)
        this->getRawSpeed(),       // e.g. " 100" (5 characters)
        this->getTargetSpeed(),
        this->getCurrent(),        // e.g. " 1.0" (5 characters)
        this->getTemperature(),
        dm3519_error_code_to_string(this->getStatus()).c_str()
    );
    return motor_info;
}
