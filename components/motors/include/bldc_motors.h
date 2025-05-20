#ifndef __BLDC_MOTORS_H
#define __BLDC_MOTORS_H


// #include <stdio.h>
#include <stdint.h>
#include "pid.h"
#include "driver/twai.h"

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

template <typename T>
class controller_t
{
private:
    PID_CONTROLLER pid_loop;
    controller_t* nextController;
    _iq* fbk;

public:
    controller_t(T* fbk) {
        this->fbk = _iQ(fbk);
        this->nextController = nullptr;
        pid_loop = {
            PID_TERM_DEFAULTS,
            PID_PARAM_DEFAULTS,
            PID_DATA_DEFAULTS
        };
    }

    ~controller_t() = default;

    template <typename U>
    void setNextController(controller_t<U>* nextController) {
        this->nextController = nextController;
    }

    void setPIDParameters(float Kp, float Ki, float Kd, float Kr, float Km, float Umax, float Umin) {
        pid_loop.param.Kp = _IQ(Kp);
        pid_loop.param.Ki = _IQ(Ki);
        pid_loop.param.Kd = _IQ(Kd);
        pid_loop.param.Kr = _IQ(Kr);
        pid_loop.param.Km = _IQ(Km);
        pid_loop.param.Umax = _IQ(Umax);
        pid_loop.param.Umin = _IQ(Umin);
    }

    template <typename U>
    _iq calOutput(T ref) {
        pid_loop.term.Ref = _IQ(ref);
        pid_loop.term.Fbk = *fbk;
        PID_MACRO(pid_loop);
        if(nextController){
            return nextController->calOutput(pid_loop.term.Out);
        }
        return pid_loop.term.Out;
    }
};

class can_channel_t; // Forward declaration of can_channel_t class

class base_motor_t
{

private:
    bool enabled = false; // Flag to check if the motor is enabled
    uint8_t motor_id;

protected:
    int16_t raw_speed;          // Raw speed of motor in counts
    int16_t raw_current;        // Raw current of motor in counts
    int16_t raw_angle;          // Raw angle of motor in counts
    int8_t status;              // Status of motor

    int16_t target_speed;   // Raw target speed of motor
    int16_t target_angle;   // Raw target angle of motor
    _iq temperature;      // Temperature of motor in Celsius

    _iq scale_angle = _IQdiv(_IQ(360.0), _IQ(8192.0)); // Scaling factor for angle
    _iq scale_current = _IQdiv(_IQ(20.0), _IQ(16384.0)); // Scaling factor for current

    can_channel_t* can_channel;

    PID_CONTROLLER speed_pid = {
        PID_TERM_DEFAULTS,
        PID_PARAM_DEFAULTS,
        PID_DATA_DEFAULTS
    };

    PID_CONTROLLER* angle_pid;

    int16_t controlOutput;

public:
    base_motor_t(uint8_t motor_id);
    virtual ~base_motor_t();

    virtual void parseData(const uint8_t* data) = 0; // Parse the data received from the motor

    void enable()                   {this->enabled = true; }    // Enable the motor
    void disable()                  {this->enabled = false; }   // Disable the motor
    
    void setMotorId(uint8_t motor_id)   {this->motor_id = motor_id;}
    void initAnglePID();

    uint8_t getMotorId() const          {return this->motor_id;}    
    int16_t getRawAngle() const         {return this->raw_angle;}

    int16_t getRawSpeed() const         {return this->raw_speed;}
    int16_t* getRawSpeedPtr()           {return &this->raw_speed;}

    int16_t getRawCurrent() const       {return this->raw_current;}
    int16_t getTargetSpeed() const      {return this->target_speed;}
    int8_t getStatus() const            {return this->status;}
    int16_t getControlOutput() const    {return this->controlOutput;}

    int16_t* getTargetSpeedPtr()        {return &this->target_speed;}
    
    float getCurrent() const            {return _IQtoF(_IQmpy(_IQ(this->raw_current), scale_current));}  // in Amps
    float getTemperature() const        {return _IQtoF(this->temperature);} // in Celsius
    float getAngle() const              {return _IQtoF(_IQmpy(_IQ(this->raw_angle), scale_angle));} // in degrees

    void setRawSpeed(int16_t raw_speed)         {this->raw_speed = raw_speed;} // Set raw speed of motor
    void setRawCurrent(int16_t current)         {this->raw_current = current;} // Set raw current of motor
    void setTargetSpeed(int16_t target_speed)   {this->target_speed = target_speed;}
    void setTargetAngle(int16_t target_angle)   {this->target_angle = target_angle;} // Set target angle of motor

    void setCanChannel(can_channel_t* can_channel) {this->can_channel = can_channel;} // Set the CAN channel for the motor

    char motor_info[256];

    /**
     * @brief Sets the parameters for the speed PID controller
     * 
     * @param Kp Proportional gain coefficient
     * @param Ki Integral gain coefficient
     * @param Kd Derivative gain coefficient
     * @param Kr Reference weight coefficient
     * @param Km Derivative weight coefficient
     * @param Umax Maximum limit for the controller output
     * @param Umin Minimum limit for the controller output
     * 
     * @note All parameters are in fixed-point _iq format
     */
    void setPIDParameters(float Kp, float Ki, float Kd, float Kr, float Km, float Umax, float Umin) {
        speed_pid.param.Kp = _IQ(Kp);
        speed_pid.param.Ki = _IQ(Ki);
        speed_pid.param.Kd = _IQ(Kd);
        speed_pid.param.Kr = _IQ(Kr);
        speed_pid.param.Km = _IQ(Km);
        speed_pid.param.Umax = _IQ(Umax);
        speed_pid.param.Umin = _IQ(Umin);
    }

    void setPIDTerminals(float c1, float c2) {
        speed_pid.term.c1 = _IQ(c1);
        speed_pid.term.c2 = _IQ(c2);
    }

    int16_t calOutput();

    virtual char* getMotorInfo();

};

#endif // __BLDC_MOTORS_H