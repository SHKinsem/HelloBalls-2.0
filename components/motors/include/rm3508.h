#ifndef __RM3508_H
#define __RM3508_H

#include "rm2006.h"

class rm3508_t : public rm2006_t
{
public:
    rm3508_t(uint8_t motor_id) : rm2006_t(motor_id) {
        // Constructor implementation if needed
    }

    ~rm3508_t() {
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
#endif // __RM3508_H