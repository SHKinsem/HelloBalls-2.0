#ifndef __RM2006_H
#define __RM2006_H


#include "bldc_motors.h"

class rm2006_t : public base_motor_t
{
private:
    _iq gear_ratio = _IQ(1.0);
    _iq shaft_angle = _IQ(0.0);
    _iq shaft_speed = _IQ(0.0);
public:
    rm2006_t(uint8_t motor_id) : base_motor_t(motor_id) {
        // Constructor implementation if needed
    }

    ~rm2006_t() {
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


#endif // __RM2006_H