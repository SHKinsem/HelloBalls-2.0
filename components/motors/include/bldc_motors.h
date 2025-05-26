#ifndef __BLDC_MOTORS_H
#define __BLDC_MOTORS_H


// #include <stdio.h>
#include <stdint.h>
#include "pid.h"
#include "driver/twai.h"
#include "esp_log.h"

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

 
class base_controller_t{
protected:
    PID_CONTROLLER pid_loop;
    bool reverse_fbk = false; // Flag to reverse feedback if needed
    _iq error; // For debugging purposes, can be removed if not needed
    base_controller_t* nextController;

public:

    int16_t debug; // For debugging purposes, can be removed if not needed

    base_controller_t() {
        pid_loop = {
            PID_TERM_DEFAULTS,
            PID_PARAM_DEFAULTS,
            PID_DATA_DEFAULTS
        };
    }

    virtual ~base_controller_t() = default;

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
        pid_loop.param.Kp = _IQ(Kp);
        pid_loop.param.Ki = _IQ(Ki);
        pid_loop.param.Kd = _IQ(Kd);
        pid_loop.param.Kr = _IQ(Kr);
        pid_loop.param.Km = _IQ(Km);
        pid_loop.param.Umax = _IQ(Umax);
        pid_loop.param.Umin = _IQ(Umin);
    }
    virtual _iq calOutput(_iq ref) = 0;

    void setReverseFeedback(bool reverse) {
        reverse_fbk = reverse; // Set the flag to reverse feedback
    }


    void reset() {
        pid_loop.data = PID_DATA_DEFAULTS; // Reset PID data
        pid_loop.term.Out = _IQ(0); // Reset output
    }

    float getError() const {
        return _IQtoF(error); // Return error in float format
    }

    base_controller_t* getTailController() {
        base_controller_t* current = this;
        while(current->nextController != nullptr) {
            current = current->nextController; // Traverse to the end of the chain
        }
        return current; // Return the last controller in the chain
    }

    void setNextController(base_controller_t* nextController) {
        base_controller_t* tail = getTailController(); // Get the last controller in the chain
        tail->nextController = nextController; // Set the next controller in the chain
        ESP_LOGI("Controller", "Next controller set");
    }

};

template <typename T>
class controller_t : public base_controller_t
{
private:
    T* fbk;
    base_controller_t* nextController;
public:
    controller_t(T* fbk) {
        this->fbk = fbk;
        this->nextController = nullptr;
    }

    ~controller_t() = default;

    _iq calOutput(_iq ref) override {
        pid_loop.term.Ref = ref;
        if(reverse_fbk){
            pid_loop.term.Fbk = _IQ(-(*fbk)); // Reverse feedback if needed
        } else {
            pid_loop.term.Fbk = _IQ(*fbk); // Use feedback as is
        }

        PID_MACRO(pid_loop);

        #ifdef DEBUG
        error = pid_loop.term.Ref - pid_loop.term.Fbk; // Calculate error for debugging
        debug = _IQint(pid_loop.term.Out); // Store output for debugging
        #endif
        
        if(nextController){
            return nextController->calOutput(pid_loop.term.Out);
        }
        return pid_loop.term.Out;
    }

    T* getFeedbackPtr() {
        return fbk; // Return pointer to feedback variable
    }
};

class can_channel_t; // Forward declaration of can_channel_t class

class base_motor_t
{

private:
    uint8_t motor_id;

protected:
    bool enabled = false; // Flag to check if the motor is enabled
    int16_t raw_speed;          // Raw speed of motor in counts
    int16_t raw_current;        // Raw current of motor in counts
    int16_t raw_angle;          // Raw angle of motor in counts
    int8_t status;              // Status of motor

    int16_t target_speed;   // Raw target speed of motor
    float target;
    _iq temperature;      // Temperature of motor in Celsius

    _iq scale_angle = _IQdiv(_IQ(360.0), _IQ(8192.0)); // Scaling factor for angle
    _iq scale_current = _IQdiv(_IQ(20.0), _IQ(16384.0)); // Scaling factor for current

    can_channel_t* can_channel;

    PID_CONTROLLER speed_pid = {
        PID_TERM_DEFAULTS,
        PID_PARAM_DEFAULTS,
        PID_DATA_DEFAULTS
    };

    base_controller_t* controller; // Controller for speed

    int16_t controlOutput;

public:

    int16_t debug;

    base_motor_t(uint8_t motor_id);
    virtual ~base_motor_t();

    virtual void parseData(const uint8_t* data) = 0; // Parse the data received from the motor

    void enable()                   {this->enabled = true; }    // Enable the motor
    void disable()                  {this->enabled = false; }   // Disable the motor
    
    void setMotorId(uint8_t motor_id)   {this->motor_id = motor_id;}

    uint8_t getMotorId() const          {return this->motor_id;}    
    int16_t getRawAngle() const         {return this->raw_angle;}

    int16_t getRawSpeed() const         {return this->raw_speed;}
    int16_t* getRawSpeedPtr()           {return &this->raw_speed;}

    int16_t getRawCurrent() const       {return this->raw_current;}
    int16_t getTargetSpeed() const      {return this->target_speed;}    // Deprecated, use getTarget() instead
    float getTarget() const             {return this->target;}
    int8_t getStatus() const            {return this->status;}
    int16_t getControlOutput() const    {return this->controlOutput;}

    int16_t* getTargetSpeedPtr()        {return &this->target_speed;}
    
    float getCurrent() const            {return _IQtoF(_IQmpy(_IQ(this->raw_current), scale_current));}  // in Amps
    float getTemperature() const        {return _IQtoF(this->temperature);} // in Celsius
    float getAngle() const              {return _IQtoF(_IQmpy(_IQ(this->raw_angle), scale_angle));} // in degrees

    void setRawSpeed(int16_t raw_speed)         {this->raw_speed = raw_speed;} // Set raw speed of motor
    void setRawCurrent(int16_t current)         {this->raw_current = current;} // Set raw current of motor
    void setTargetSpeed(int16_t target_speed)   {this->target_speed = target_speed;}
    void setTarget(float target)                {this->target = target;} // Set target speed of motor

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
        controller->setPIDParameters(Kp, Ki, Kd, Kr, Km, Umax, Umin);
    }

    void setNextController(base_controller_t* next_controller) {
        if (!controller) return;
        controller->setNextController(next_controller); // Set the new controller
    }

    virtual int16_t& calOutput();

    virtual char* getMotorInfo();

};

#endif // __BLDC_MOTORS_H