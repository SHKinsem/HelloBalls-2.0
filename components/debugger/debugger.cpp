#include "app_button.h"
#include "app_twai.h"
#include "ui.h"
#include "app_motors.h"
#include "bldc_motors.h"
#include "debugger.h"

#define TAG "DEBUG"
dm3519_t motor1(1); // Create an instance of base_motor_t for motor 1
int16_t output = 200;

char motor_info[256];

void motor_task(void* arg){
    // Create a task for motor control

    motor1.enable();
    motor1.setTargetSpeed(700); // Set target speed to 1000 counts
    motor1.setPIDParameters(0.7, 0.1, 0.01, 1.0, 0.0, 1000.0, -1000.0); // Set PID parameters
    uint8_t* motorData = motorDataHook(1); // Get motor data
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms
    esp_err_t status = twai_transmit(motor1.getClearErrorMessage(), pdMS_TO_TICKS(1000));
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Error twai_transmit message: %s", esp_err_to_name(status));
    }
    while (1) {
        snprintf(motor_info, sizeof(motor_info),
            "Motor %2u:\n"
            "Angle:\n\t%6.1f Degrees\n"      // Fixed width for angle
            "Speed:\n\t%5d RPM\n"            // Fixed width for RPM
            "Current:\n\t%5.1f Amps\n"       // Fixed width for current
            "Temperature:\n\t%5.1f Celsius\n"// Fixed width for temp
            "Status:\t%s",                   // Status string
            motor1.getMotorId(),
            motor1.getAngle(),          // e.g. " 360.0" (always 6 characters)
            motor1.getRawSpeed(),       // e.g. " 100" (5 characters)
            motor1.getCurrent(),        // e.g. " 1.0" (5 characters)
            motor1.getTemperature(),
            dm3519_error_code_to_string(motor1.getStatus()).c_str()
        );
        // Example: Set the target speed of the motor
        motor1.parseData(motorData); // Parse data from motor 1
        // motor1.setRawSpeed(getGlobalSpeed()); // Set raw speed to 0 counts
        output = motor1.calOutput(); // Calculate control output
        vTaskDelay(pdMS_TO_TICKS(80)); // Delay for 100 ms
        lv_textarea_set_text(ui_Screen1TextareaTextArea2, motor_info);

    }
}

void motor_task_init(){
    // Create a task for motor control
    xTaskCreatePinnedToCore(motor_task, "Motor Control Task", 4096, NULL, 5, NULL, 1); // Create the motor task on core 1
}

void debugLoggingTask(void *arg) {
    while (1) {
        // Log the current speed of the motor
        // ESP_LOGI("MC", "Target: %d, CurSpd: %d, Output: %d", motor1.getTargetSpeed(), motor1.getRawSpeed(), output);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}



// void debugHelperTask(void *arg) {
//     // Create a task for logging debug information
//     xTaskCreate(debugLoggingTask, "Debug Logging Task", 2048, NULL, 5, NULL);
    
//     // Wait for the task to complete (it won't in this case)
//     vTaskDelete(NULL);
// }