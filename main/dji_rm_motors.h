
#ifndef __DJI_MOTORS_C
#define __DJI_MOTORS_C

#include <stdio.h>
#include <stdlib.h>

#define FOUR_SLAVE_MODE
#define CAN_SPEED 1000 // 1 Mbps

#ifdef FOUR_SLAVE_MODE
    #define MOTOR_ID_MASK 0x200 // 0x201 - 0x204 for motors 1-4
#endif


#ifdef __cplusplus
extern "C" {
#endif

class can_channel_t
{
public:
    can_channel_t();
    ~can_channel_t();

    void init(int channel_id, int baud_rate);
    void start();
    void stop();
    void sendMessage(int message_id, const char *data, int data_length);
    void receiveMessage(int *message_id, char *data, int *data_length);
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
    int motor_id;
    can_channel_t *can_channel;

    int raw_speed; // Raw speed of motor in counts
    int raw_current; // Raw current of motor in counts
    float temperature; // Temperature of motor in Celsius

    void parseData(const char *data, int data_length); // Parse the data received from the motor
public:
    motor_data_t();
    ~motor_data_t();

    int getRawSpeed(); // Get raw speed of motor in counts
    int getRawCurrent(); // Get raw current of motor in counts

    float getSpeed(); // Get speed of motor in RPM
    float getCurrent(); // Get current of motor in Amps
    float getTemperature(); // Get temperature of motor in Celsius
};

class dji_rm_motor_t:
 private motor_data_t,
 private pid_controller_t
{
public:
    dji_rm_motor_t();
    ~dji_rm_motor_t();

    void init(int motor_id, can_channel_t *can_channel);
    void setConttoller(int mode);
    void enableMotor();

};


#ifdef __cplusplus
}
#endif
#endif // __DJI_MOTORS_C