#include "esp32_s3_szp.h"
#include "app_ui.h"
#include "app_button.h"
#include "app_twai.h"
#include "ui.h"
#include "app_motors.h"

base_motor_t motor1(1); // Create an instance of base_motor_t for motor 1
int16_t output = 200;

void motor_task(void* arg){
    // Create a task for motor control
    motor1.enable();
    motor1.setTargetSpeed(700); // Set target speed to 1000 counts
    motor1.setPIDParameters(0.7, 0.1, 0.01, 1.0, 0.0, 1000.0, -1000.0); // Set PID parameters
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms
    while (1) {
        // Example: Set the target speed of the motor
        twai_transmit_speed(output); // Transmit the control output
        motor1.setRawSpeed(getGlobalSpeed()); // Set raw speed to 0 counts
        output = motor1.calOutput(); // Calculate control output
        vTaskDelay(pdMS_TO_TICKS(1)); // Delay for 100 ms
    }
}

void motor_task_init(){
    // Create a task for motor control
    xTaskCreate(motor_task, "Motor Control Task", 2048, NULL, 5, NULL);
}

void debugLoggingTask(void *arg) {
    while (1) {
        // Log the current speed of the motor
        ESP_LOGI("MC", "Target: %d, CurSpd: %d, Output: %d", motor1.getTargetSpeed(), getGlobalSpeed(), output);
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 1 second
    }
}



// void debugHelperTask(void *arg) {
//     // Create a task for logging debug information
//     xTaskCreate(debugLoggingTask, "Debug Logging Task", 2048, NULL, 5, NULL);
    
//     // Wait for the task to complete (it won't in this case)
//     vTaskDelete(NULL);
// }