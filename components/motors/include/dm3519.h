#ifndef __DM3519_H
#define __DM3519_H

#include "bldc_motors.h"

#ifdef USE_DM3519

#include <string>
#include "rm3508.h"
#include "driver/twai.h"

/**
 * @brief Motor data structure for Damiao DM3519 motors.
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
 * Data[7] = Status byte        The code is listed below
 * 
 */

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

private:
    DM3519_ERROR_CODE error_code = DM3519_DISABLED; // Error code for the motor
    
    twai_message_t clr_err_msg = {};

public:
    dm3519_t(uint8_t motor_id) : rm3508_t(motor_id) {
        // Constructor implementation if needed
        clr_err_msg.extd = 0;
        clr_err_msg.rtr = 0;
        clr_err_msg.ss = 1;
        clr_err_msg.self = 0;
        clr_err_msg.dlc_non_comp = 0;
        clr_err_msg.identifier = 0x7FF;
        clr_err_msg.data_length_code = 4;
        clr_err_msg.data[0] = (motor_id << 4) | 0x00;
        clr_err_msg.data[1] = 0x00;
        clr_err_msg.data[2] = 0x55;
        clr_err_msg.data[3] = 0x3C;

    }

    ~dm3519_t() {
        // Destructor implementation if needed
    }

    void parseData(const uint8_t* data) {
        // Parse the data received from the motor
        this->raw_angle =   (uint16_t)((data[0] << 8) | data[1]); // Combine high and low byte for angle
        this->raw_speed =   (uint16_t)((data[2] << 8) | data[3]); // Combine high and low byte for speed
        this->raw_current = (uint16_t)((data[4] << 8) | data[5]); // Combine high and low byte for current
        this->temperature = _IQ(data[6]); // Temperature byte
        // this->status = data[7]; // Status byte
        this->error_code = static_cast<DM3519_ERROR_CODE>(data[7]); // Set the error code based on the status byte
    }

    DM3519_ERROR_CODE getStatus() const {
        return this->error_code; // Return the error code
    }

    twai_message_t* getClearErrorMessage() const {
        return const_cast<twai_message_t*>(&clr_err_msg);
    }
};


#endif

#endif // __DM3519_H
