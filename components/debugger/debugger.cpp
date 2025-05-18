#include "debugger.h"
#include "app_button.h"
#include "app_twai.h"
#include "ui.h"
#include "bldc_motors.h"
#include "ui_events.h"
#include "app_motors.h"
#include "esp_timer.h"
#include "stepper_motors.h"
#include "serial.h"

#define TAG "DEBUG"
int16_t output = 0;

base_motor_t* motor_ptr[4] = {nullptr, nullptr, nullptr, nullptr}; // Array of motor pointers




void motor_displayer(void* arg){
    // Create a task for motor control
    setGlobalSpeedPtr(motor_ptr[0]->getTargetSpeedPtr());
    setGlobalSpeedPtr2(motor_ptr[1]->getTargetSpeedPtr());

    motor_ptr[0]->setPIDParameters(20.0, 0.01, 0.005, 0.1, 0.1, 10000.0, -10000.0); // Set PID parameters
    motor_ptr[1]->setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 10000.0, -10000.0); // Set PID parameters

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms
    int counter = 0; // Counter for the run time
    while (1) {
        if(get_twai_running()) {
            if (counter < 5){
                lv_textarea_set_text(ui_MainMenuTextareaTextArea2, motor_ptr[0]->getMotorInfo());
            } else if (counter >= 5 && counter < 10) {
                lv_textarea_set_text(ui_MainMenuTextareaTextArea2, motor_ptr[1]->getMotorInfo());
            } else if (counter >= 10 && counter < 20) {
                lv_textarea_set_text(ui_MainMenuTextareaTextArea2, motor_ptr[2]->getMotorInfo());
            } else if (counter >= 20) {
                counter = 0; // Reset the counter
            }
            counter++;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}