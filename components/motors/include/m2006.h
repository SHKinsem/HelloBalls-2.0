#ifndef __M2006_H
#define __M2006_H


#include "bldc_motors.h"

/**
 * @brief Motor data structure for DJI M2006 motors.
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
protected:
    _iq gear_ratio = _IQ(36.0/1.0);
    _iq shaft_angle = _IQ(0.0);
    _iq shaft_speed = _IQ(0.0);
    int16_t loopCounter = 0;
    int16_t prev_raw_angle = 0;
    int16_t maxBump = 2214;
    // Max RPM: ~16200, 16200/60 = 270 RPS, 270*8192 = 221,340 Counts/second
    // Max Bump = 221,340 Counts/second * 0.01 seconds = 2213 counts
public:
    m2006_t(uint8_t motor_id) : base_motor_t(motor_id) {
        scale_current = _IQdiv(_IQ(10.0), _IQ(10000.0)); // Scaling factor for current
        temperature = -1; // M2006 temperature is not available
        status = 0; // Initialize raw status to 0
        loopCounter = 0;
    }

    ~m2006_t() {
        // Destructor implementation if needed
    }

    void parseData(const uint8_t* data) override {
        // Parse the data received from the motor
        this->raw_angle =   (uint16_t)((data[0] << 8) | data[1]); // Combine high and low byte for angle
        this->raw_speed =   (uint16_t)((data[2] << 8) | data[3]); // Combine high and low byte for speed
        this->raw_current = (uint16_t)((data[4] << 8) | data[5]); // Combine high and low byte for current
        // this->temperature = data[6]; // Temperature byte
        // this->raw_status = data[7]; // Status byte
        if(raw_angle - prev_raw_angle > maxBump){
            loopCounter--;
        } else if (prev_raw_angle - raw_angle > maxBump) {
            loopCounter++;
        }
        prev_raw_angle = raw_angle;
    }

    void resetCounter() {loopCounter = 0;}
    int16_t getCounter() {return loopCounter;}
    _iq calShaftAngle() {
        // Calculate the shaft angle based on the raw angle and loop counter
        _iq angle = _IQmpy(_IQ(raw_angle), scale_angle) + _IQ(360 * loopCounter); 
        this->shaft_angle = _IQdiv(angle, gear_ratio); // Divide by gear ratio
        return this->shaft_angle;
    }

    float getShaftAngle()       {return _IQtoF(calShaftAngle());}
    _iq* getShaftAnglePtr()     {return &this->shaft_angle;}

    char* getMotorInfo() override {
        snprintf(motor_info, sizeof(motor_info),
            "Motor ID: %1u:\n"
            "Angle:\n\t%3.2f Degrees\n" 
            "Shaft:\n\t%5.2f\n"
            "Speed:\n\t%5d RPM\n"
            "TargetSpeed:\n\t%5d RPM\n"
            "Current:\n\t%2.2f Amps\n",
            this->getMotorId(),
            this->getAngle(),
            this->getShaftAngle(),
            this->getRawSpeed(),
            this->getTargetSpeed(),
            this->getCurrent()
        );
        return motor_info;
    }
};


#endif // __M2006_H