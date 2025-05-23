#ifndef __M3508_H
#define __M3508_H

#include "m2006.h"

/**
 * @brief Motor data structure for DJI M3508 motors.
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

class m3508_t : public m2006_t
{
public:
    m3508_t(uint8_t motor_id) : m2006_t(motor_id) {
        // Constructor implementation if needed
    }

    ~m3508_t() {
        // Destructor implementation if needed
    }

    void parseData(const uint8_t* data) override {
        // Parse the data received from the motor
        this->raw_angle =   (uint16_t)((data[0] << 8) | data[1]); // Combine high and low byte for angle
        this->raw_speed =   (uint16_t)((data[2] << 8) | data[3]); // Combine high and low byte for speed
        this->raw_current = (uint16_t)((data[4] << 8) | data[5]); // Combine high and low byte for current
        this->temperature = _IQ(data[6]); // Temperature byte
        this->status = data[7]; // Status byte
    }

    char* getMotorInfo() override {
        snprintf(motor_info, sizeof(motor_info),
            "Motor ID: %1u:\n"
            "Angle:\n\t%3.2f Degrees\n" 
            // "Shaft:\n\t%5.2f\n"
            "Speed:\n\t%5d RPM\n"
            "TargetSpeed:\n\t%5d RPM\n"
            "Current:\n\t%2.2f Amps\n",
            this->getMotorId(),
            this->getAngle(),
            // this->getShaftAngle(), // Added call to getShaftAngle()
            this->getRawSpeed(),
            this->getTargetSpeed(),
            this->getCurrent()
        );
        return motor_info;
    }
};
#endif // __M3508_H