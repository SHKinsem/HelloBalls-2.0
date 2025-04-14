#include "app_button.h"
#include "app_twai.h"
#include "ui.h"
#include "app_motors.h"
#include "bldc_motors.h"
#include "debugger.h"
#include "ui_events.h"

#define TAG "DEBUG"
// rm2006_t motor1(1); // Create an instance of base_motor_t for motor 1
dm3519_t motor1(1);
int16_t output = 200;
uint8_t* motorData;

void motor_displayer(void* arg){
    // Create a task for motor control

    motor1.enable();
    // motor1.setTargetSpeed(getGlobalSpeed()); // Set target speed to 1000 counts
    setGlobalSpeedPtr(motor1.getTargetSpeedPtr());
    motor1.setPIDParameters(1.0, 0.1, 0.2, 1.0, 0.0, 1000.0, -1000.0); // Set PID parameters
    motorData = motorDataHook(1); // Get motor data
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200)); // Refresh every 100 ms
        lv_textarea_set_text(ui_MainMenuTextareaTextArea2, motor1.getMotorInfo()); // Update the UI with motor info
    }
}


void debugLoggingTask(void *arg) {
    while (1) {
        // Log the current speed of the motor
        // ESP_LOGI("MC", "Target: %d, CurSpd: %d, Output: %d", *getGlobalSpeed(), motor1.getRawSpeed(), output);
        int counter = 10000;
        twai_transmit_speed(0); // Transmit the speed to the motor
        motor1.parseData(motorData); // Parse data from motor 1
        if(*get_button_state_ptr()) {
            lv_obj_set_style_bg_color(ui_MainMenuButtonTestButton, lv_color_hex(0xFF0085), LV_PART_MAIN | LV_STATE_DEFAULT);
            while(get_twai_running() && counter > 0) {
                // Log the current speed of the motor
                // ESP_LOGI(TAG, "Output: %d", output);
                counter--;
                motor1.parseData(motorData);
                output = motor1.calOutput(); // Calculate control output
                twai_transmit_speed(output); // Transmit the speed to the motor
                vTaskDelay(pdMS_TO_TICKS(1));
                // Run the motor for total of 5000ms
            }
            *get_button_state_ptr() = false; // Reset button state
            lv_obj_set_style_bg_color(ui_MainMenuButtonTestButton, lv_color_hex(0x838383), LV_PART_MAIN | LV_STATE_DEFAULT);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}



void motor_task_init(){
    // Create a task for motor control
    xTaskCreatePinnedToCore(motor_displayer, "Motor Control Task", 4096, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms
    xTaskCreatePinnedToCore(debugLoggingTask, "Debug Logging Task", 4096, NULL, 5, NULL, 1);
}