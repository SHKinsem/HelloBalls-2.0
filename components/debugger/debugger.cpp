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

can_channel_t can_channel(0, TWAI_TX_PIN, TWAI_RX_PIN); // Create a CAN channel instance
dm3519_t test_motor(1); // Create an instance of the motor



void motor_displayer(void* arg){
    // Create a task for motor control
    setGlobalSpeedPtr(motor_ptr[0]->getTargetSpeedPtr());
    setGlobalSpeedPtr2(motor_ptr[1]->getTargetSpeedPtr());

    motor_ptr[0]->setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 10000.0, -10000.0); // Set PID parameters
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

#define RUN_MOTOR_PERIOD 10000
#define MESSAGE_FREQUENCY 1 // Frequency of sending messages in ms
#define MAX_COUNTER RUN_MOTOR_PERIOD / MESSAGE_FREQUENCY // Maximum counter value

void debugLoggingTask(void *arg) {
    while (1) {
        // Only transmit zeros when button is not pressed, and no serial control is active
        if (!*get_button_state_ptr()) {
            // Skip sending zero messages - serialWheelControlTask handles this now
            // twai_transmit_speed(0, 0, 0, 0); <- removed to avoid conflict
        }
        
        if(*get_button_state_ptr()) {

            motor_ptr[0]->enable(); // Use the pointer to enable the motor
            motor_ptr[1]->enable(); // Use the pointer to enable the motor
            ESP_LOGI(TAG, "Motor started, output: %d", output);
            lv_obj_set_style_bg_color(ui_MainMenuButtonTestButton, lv_color_hex(0xFF0085), LV_PART_MAIN | LV_STATE_DEFAULT);
            TickType_t xLastWakeTime = xTaskGetTickCount(); // Get the current tick count

            for (int counter = 0; counter < MAX_COUNTER; counter++) {
                if (!get_twai_running()) break;
                if(counter % 1000 == 0) ESP_LOGI(TAG, "Counter: %d", counter);
                twai_transmit_speed(motor_ptr[0]->calOutput(), motor_ptr[1]->calOutput(), 0, 0); // Transmit the speed to the motor
                // ESP_LOGI(TAG, "O: %d, R: %d, T: %d", output, motor_ptr[0]->getRawSpeed(), motor_ptr[0]->getTargetSpeed());
                xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MESSAGE_FREQUENCY)); // Delay until the next period
                // vTaskDelay(pdMS_TO_TICKS(MESSAGE_FREQUENCY)); // Delay for the message frequency
            }
            motor_ptr[0]->disable();
            motor_ptr[1]->disable();
            
            *get_button_state_ptr() = false; // Reset button state
            lv_obj_set_style_bg_color(ui_MainMenuButtonTestButton, lv_color_hex(0x838383), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        // ESP_LOGI(TAG, "R: %d, T: %d", motor_ptr[0]->getRawSpeed(), motor_ptr[0]->getTargetSpeed());

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void measure_important_function(void) {
    const unsigned MEASUREMENTS = 5000;
    uint64_t start = esp_timer_get_time();

    for (int retries = 0; retries < MEASUREMENTS; retries++) {
        twai_transmit_speed(motor_ptr[0]->getControlOutput(), motor_ptr[1]->getControlOutput(), 0, 0); // Transmit the speed to the motor
    }

    uint64_t end = esp_timer_get_time();

    printf("%u iterations took %llu milliseconds (%llu microseconds per invocation)\n",
           MEASUREMENTS, (end - start)/1000, (end - start)/MEASUREMENTS);
}

void serialWheelControlTask(void *arg) {
    ESP_LOGI(TAG, "Serial wheel control task started");
    // Setup for serial activity detection
    rx_message_t* rx_msg = get_rx_message();
    task_state_t* task_state = getTaskState(); // Get the task state pointer
    
    while (1) {
        if (*task_state == RECEIVING) { test_motor.enable(); } // Enable the motor if receiving data
        else { test_motor.disable(); } // Disable the motor if not receiving data

        test_motor.setTargetSpeed(rx_msg->wheel1_speed); // Set the target speed for the motor
        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

void motor_task_init(){
    can_channel.reg_motor(&test_motor); // Register the motor with the CAN channel

    can_channel.start(); // Start the CAN channel
    test_motor.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters
    // Create the new serial wheel control task
    xTaskCreatePinnedToCore(serialWheelControlTask, "Serial Wheel Control", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 0);
}