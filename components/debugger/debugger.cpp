#include "app_button.h"
#include "app_twai.h"
#include "ui.h"
#include "app_motors.h"
#include "bldc_motors.h"
#include "debugger.h"
#include "ui_events.h"

#define TAG "DEBUG"
rm2006_t motor1(1); // Create an instance of base_motor_t for motor 1
int16_t output = 200;

char motor_info[256];

void motor_displayer(void* arg){
    // Create a task for motor control

    motor1.enable();
    // motor1.setTargetSpeed(getGlobalSpeed()); // Set target speed to 1000 counts
    setGlobalSpeedPtr(motor1.getTargetSpeedPtr());
    motor1.setPIDParameters(1.0, 0.5, 0.1, 1.0, 0.0, 5000.0, -5000.0); // Set PID parameters
    uint8_t* motorData = motorDataHook(1); // Get motor data
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms

    while (1) {
        motor1.parseData(motorData); // Parse data from motor 1
        output = -motor1.calOutput(); // Calculate control output
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz control loop
        lv_textarea_set_text(ui_MainMenuTextareaTextArea2, motor1.getMotorInfo()); // Update the UI with motor info
    }
}

void motor_task_init(){
    // Create a task for motor control
    xTaskCreatePinnedToCore(motor_displayer, "Motor Control Task", 4096, NULL, 5, NULL, 1); // Create the motor task on core 1
}

void debugLoggingTask(void *arg) {
    while (1) {
        // Log the current speed of the motor
        // ESP_LOGI("MC", "Target: %d, CurSpd: %d, Output: %d", *getGlobalSpeed(), motor1.getRawSpeed(), output);
        int counter = 1000;
        if(get_twai_running() && *get_button_state_ptr()) {
            while(counter > 0) {
                // Log the current speed of the motor
                // ESP_LOGI(TAG, "Target: %d, CurSpd: %d, Output: %d", *getGlobalSpeed(), motor1.getRawSpeed(), output);
                counter--;
                twai_transmit_speed(output); // Transmit the speed to the motor`
                vTaskDelay(pdMS_TO_TICKS(5));
                // Run the motor for total of 5000ms
            }
            *get_button_state_ptr() = false; // Reset button state
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}



// void debugHelperTask(void *arg) {
//     // Create a task for logging debug information
//     xTaskCreate(debugLoggingTask, "Debug Logging Task", 2048, NULL, 5, NULL);
    
//     // Wait for the task to complete (it won't in this case)
//     vTaskDelete(NULL);
// }