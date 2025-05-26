#include <stdint.h>
#include "pid.h"
// #include "driver/twai.h"
#include "esp_log.h"

#ifndef CONTROLLER_H
#define CONTROLLER_H

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
    }

};

template <typename T>
class controller_t : public base_controller_t
{
private:
    T* fbk;
public:
    controller_t(T* fbk) {
        this->fbk = fbk;
        this->nextController = nullptr;
    }

    ~controller_t() override {
        // Destructor to clean up resources if needed
        base_controller_t* next = nextController;
        while (next != nullptr) {
            base_controller_t* temp = next;
            next = next->nextController; // Move to the next controller
            delete temp; // Delete the current controller
        }
    }

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

#endif // CONTROLLER_H