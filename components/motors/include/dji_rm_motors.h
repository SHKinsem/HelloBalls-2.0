#ifndef __DJI_MOTORS_H
#define __DJI_MOTORS_H

#include <stdio.h>
#include <stdlib.h>
#include "pid.h"    // PID Contorller from TI
#include "IQmathLib.h"
#include "driver/twai.h"

#define USE_DM3519
#define FOUR_SLAVE_MODE
#define CAN_SPEED 1000 // 1 Mbps

#ifdef FOUR_SLAVE_MODE
    #define MOTOR_ID_MASK 0x200 // 0x201 - 0x204 for motors 1-4
#endif

// static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps
// static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

#ifdef __cplusplus
extern "C" {
#endif

// Specific for ESP32
class can_channel_t
{
private:
    const uint8_t channel_id;
    const uint32_t baud_rate;
    const gpio_num_t gpio_rx;
    const gpio_num_t gpio_tx;
    uint8_t state;
    uint8_t *tx_buffer;
    uint8_t **rx_buffer;
public:
    can_channel_t(gpio_num_t gpio_tx, gpio_num_t gpio_rx, uint8_t channel_id, uint32_t baud_rate)
        :gpio_rx(gpio_tx), 
        gpio_tx(gpio_rx), 
        channel_id(channel_id), 
        baud_rate(baud_rate) {
        const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
        const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_tx, gpio_rx, TWAI_MODE_NORMAL);
        // Install the twai driver
        ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

        tx_buffer = new uint8_t[8];
        rx_buffer = new uint8_t*[4];
    }
    ~can_channel_t();

    // void init(int channel_id, int baud_rate);
    void start();
    void stop();
    void sendMessage(int message_id, const char *data, int data_length);
    void receiveMessage(int *message_id, char *data, int *data_length);
    uint8_t getState() {return this->state;}
    uint8_t* getMotorRx(uint8_t motor_id) {return this->rx_buffer[motor_id];}
    uint8_t getMotorTx(uint8_t motor_id) {return this->tx_buffer[motor_id];}
};

class pid_controller_t
{
private:
    float k_p;
    float k_i;
    float k_d;
    float k_i_max;
    
public:
    pid_controller_t();
    ~pid_controller_t();

    void init();
    void setMotorSpeed(int motor_id, int speed);
    void getMotorStatus(int motor_id, int *status);
    void startMotor(int motor_id, int speed);
    void stopMotor(int motor_id);
};


/**
 * @brief Motor data structure for DJI RM motors.
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
 * Data[7] = Status byte        Depending on the motor
 * 
 */

class motor_data_t
{
private:
    uint8_t motor_id;
    can_channel_t *can_channel;
    pid_controller_t *Controller;

    int raw_speed;          // Raw speed of motor in counts
    int raw_current;        // Raw current of motor in counts
    int raw_angle;          // Raw angle of motor in counts
    int raw_status;         // Raw status of motor
    int raw_target_speed;   // Raw target speed of motor
    int raw_target_angle;   // Raw target angle of motor
    _iq24 temperature;      // Temperature of motor in Celsius

    const _iq24 scale_angle = _IQ24div(_IQ24(360.0), _IQ24(8191.0)); // Scaling factor for angle
    const _iq24 scale_current = _IQ24div(_IQ24(20.0), _IQ24(16384.0)); // Scaling factor for current

    void parseData(const char *data, int data_length); // Parse the data received from the motor
public:
    motor_data_t(can_channel_t *can_channel, int motor_id);
    virtual ~motor_data_t();

    void setMotorId(int motor_id)   {this->motor_id = motor_id;}
    void setRawSpeed(int speed)     {this->raw_speed = speed;}
    void setRawCurrent(int current) {this->raw_current = current;}

    int getMotorId()                {return this->motor_id;}    // Get motor ID
    int getRawAngle()               {return this->raw_angle;}   // Get raw angle of motor in counts
    int getRawSpeed()               {return this->raw_speed;}   // Get raw speed of motor in RPM
    int getRawCurrent()             {return this->raw_current;} // Get raw current of motor in counts

    _iq24 getCurrent()              {return _IQ24mpy(_IQ24(this->raw_current), scale_current);}  // Get current of motor in Amps
    _iq24 getTemperature()          {return this->temperature;} // Get temperature of motor in Celsius
    _iq24 getAngle()                {return _IQ24mpy(_IQ24(this->raw_angle), scale_angle);} // Get angle of motor in degrees
};

class dji_rm_motor_t:
 public motor_data_t
{
public:
    dji_rm_motor_t();
    ~dji_rm_motor_t();

    void init(int motor_id, can_channel_t *can_channel);
    void setController(int mode);
    void enableMotor();
};


#ifdef __cplusplus
}
#endif
#endif // __DJI_MOTORS_H