#ifndef __M2006_H
#define __M2006_H


#include "bldc_motors.h"

/**
 * @brief Motor data structure for DJI M2006 motors.
 * 
 * This structure contains the data received from the motor over CAN bus.
 * The data is received in the following format:
 * 
 * Data[0] = Angle high byte    Range: [0, 8191]
 * Data[1] = Angle low byte     Mapped to [0, 360] degrees
 * Data[2] = Speed high byte    In RPM
 * Data[3] = Speed low byte
 * Data[4] = Current high byte  Range: [-10000, 10000]
 * Data[5] = Current low byte   Mapped to [-10, 10] Amps, depending on the motor
 * Data[6] = Null byte          Not used
 * Data[7] = Null byte          Not used
 * 
 */


class m2006_t : public base_motor_t
{
private:
    _iq gear_ratio = _IQ(1.0);
    _iq shaft_angle = _IQ(0.0);
    _iq shaft_speed = _IQ(0.0);
public:
    m2006_t(uint8_t motor_id) : base_motor_t(motor_id) {
        scale_current = _IQdiv(_IQ(10.0), _IQ(10000.0)); // Scaling factor for current
        temperature = -1; // M2006 temperature is not available
        status = 0;
    }

    ~m2006_t() {
        // Destructor implementation if needed
    }

    void parseData(const uint8_t* data) {
        // Parse the data received from the motor
        this->raw_angle =   (uint16_t)((data[0] << 8) | data[1]); // Combine high and low byte for angle
        this->raw_speed =   (uint16_t)((data[2] << 8) | data[3]); // Combine high and low byte for speed
        this->raw_current = (uint16_t)((data[4] << 8) | data[5]); // Combine high and low byte for current
        // this->temperature = data[6]; // Temperature byte
        // this->raw_status = data[7]; // Status byte
    }

    // char* getMotorInfo() override;
};


#endif // __M2006_H