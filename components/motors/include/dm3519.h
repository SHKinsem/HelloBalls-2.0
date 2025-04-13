#ifndef __DM3519_H
#define __DM3519_H

#include "bldc_motors.h"
#include "rm3508.h"

#ifdef USE_DM3519

#include <string>
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

class dm3519_t : public rm3508_t
{
public:
    dm3519_t(uint8_t motor_id) : rm3508_t(motor_id) {
        // Constructor implementation if needed
    }

    ~dm3519_t() {
        // Destructor implementation if needed
    }

    void parseData(const uint8_t data[]) override {
        // Parse the data received from the motor
        this->raw_angle = (data[0] << 8) | data[1]; // Combine high and low byte for angle
        this->raw_speed = (data[2] << 8) | data[3]; // Combine high and low byte for speed
        this->raw_current = (data[4] << 8) | data[5]; // Combine high and low byte for current
        this->temperature = data[6]; // Temperature byte
        this->raw_status = data[7]; // Status byte
    }
};
    
#endif

#endif // __DM3519_H
