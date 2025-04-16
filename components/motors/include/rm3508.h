#ifndef __RM3508_H
#define __RM3508_H

#include "rm2006.h"

/**
 * @brief Motor data structure for DJI RM3508 motors.
 * 
 * This structure contains the data received from the motor over CAN bus.
 * The data is received in the following format:
 * 
 * Data[0] = Angle high byte    Range: [0, 8191]
 * Data[1] = Angle low byte     Mapped to [0, 360] degrees
 * Data[2] = Speed high byte    In RPM
 * Data[3] = Speed low byte
 * Data[4] = Current high byte  Range: [-16384, 16384]
 * Data[5] = Current low byte   Mapped to [-20, 20] Amps, depending on the motor
 * Data[6] = Temperature byte   In Celsius
 * Data[7] = Null        Depending on the motor
 * 
 */

class rm3508_t : public rm2006_t
{
public:
    rm3508_t(uint8_t motor_id) : rm2006_t(motor_id) {
        // Constructor implementation if needed
    }

    ~rm3508_t() {
        // Destructor implementation if needed
    }

    void parseData(const uint8_t* data) {
        // Parse the data received from the motor
        this->raw_angle =   (uint16_t)((data[0] << 8) | data[1]); // Combine high and low byte for angle
        this->raw_speed =   (uint16_t)((data[2] << 8) | data[3]); // Combine high and low byte for speed
        this->raw_current = (uint16_t)((data[4] << 8) | data[5]); // Combine high and low byte for current
        this->temperature = _IQ(data[6]); // Temperature byte
        this->status = data[7]; // Status byte
        calOutput(); // Calculate control output
    }
};
#endif // __RM3508_H