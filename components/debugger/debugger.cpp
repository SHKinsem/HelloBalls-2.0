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
    
    // Wait a bit to ensure all systems are initialized
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Define task frequency - 1000Hz (1ms period)
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms = 1000Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Variables for serial connection monitoring
    bool prev_had_connection = false;
    bool current_has_connection = false;
    const uint32_t CONNECTION_TIMEOUT_MS = 1000; // 1 second timeout
    
    // Setup for serial activity detection
    rx_message_t* rx_msg = get_rx_message();
    uint32_t last_update_time = xTaskGetTickCount();
    uint32_t message_counter = 0;
    
    // Variable to track last received serial message
    uint8_t last_signal_counter = rx_msg->machine_state;
    
    while (1) {
        // Check if we have fresh serial data
        if (last_signal_counter != rx_msg->machine_state) {
            // Update our record of the last signal counter
            last_signal_counter = rx_msg->machine_state;
            
            // Fresh data detected
            last_update_time = xTaskGetTickCount();
            current_has_connection = true;
            
            // Log reception of new speed command periodically
            if (message_counter++ % 500 == 0) {
                ESP_LOGI(TAG, "New speed command: W1=%d, W2=%d, Signal=0x%02X", 
                    rx_msg->wheel1_speed, rx_msg->wheel2_speed, rx_msg->machine_state);
            }
        } else {
            // Check if we've exceeded the timeout
            uint32_t elapsed = xTaskGetTickCount() - last_update_time;
            current_has_connection = (elapsed < pdMS_TO_TICKS(CONNECTION_TIMEOUT_MS));
        }
        
        // Get outputs
        int16_t output1 = 0;
        int16_t output2 = 0;
        
        if (get_twai_running()) {
            if (current_has_connection) {
                // Connection is active - enable motors and apply wheel speeds from serial
                motor_ptr[0]->enable();
                motor_ptr[1]->enable();
                motor_ptr[0]->setTargetSpeed(rx_msg->wheel1_speed);
                motor_ptr[1]->setTargetSpeed(rx_msg->wheel2_speed);
                
                // Log on connection state change
                if (!prev_had_connection) {
                    ESP_LOGI(TAG, "Serial connection restored, resuming normal operation");
                }
            } else {
                // Connection lost, disable the motors
                motor_ptr[0]->disable();
                motor_ptr[1]->disable();
                
                // Log on connection state change
                if (prev_had_connection) {
                    ESP_LOGI(TAG, "Serial connection lost, disabling motors");
                }
            }
            
            // Always calculate and send control outputs regardless of connection state
            output1 = motor_ptr[0]->calOutput();
            output2 = motor_ptr[1]->calOutput();
            
            // Send TWAI message with calculated outputs - always at 1000Hz
            twai_transmit_speed(output1, output2, 0, 0);
            
            // Log status periodically (once per second)
            static uint32_t counter = 0;
            if (counter++ % 1000 == 0) {
                ESP_LOGI(TAG, "Serial wheels: T1=%d, T2=%d, O1=%d, O2=%d, Connected=%d", 
                    motor_ptr[0]->getTargetSpeed(), motor_ptr[1]->getTargetSpeed(),
                    output1, output2, current_has_connection);
            }
        }
        
        // Remember connection state for next iteration
        prev_had_connection = current_has_connection;
        
        // Precise timing for 1000Hz frequency
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motor_task_init(){
    twai_init();
    for(int i = 0; i < 4; i++) {
        motor_ptr[i] = get_motor_ptr(i + 1); // Initialize all motor pointers to null
    }

    // Create a task for motor control
    xTaskCreatePinnedToCore(motor_displayer, "Motor displayer Task", 4096, NULL, 3, NULL, tskNO_AFFINITY);
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms
    xTaskCreatePinnedToCore(debugLoggingTask, "Debug Logging Task", 4096, NULL, configMAX_PRIORITIES - 4, NULL, 0);
    
    // Create the new serial wheel control task
    xTaskCreatePinnedToCore(serialWheelControlTask, "Serial Wheel Control", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 0);
    
    // Start TWAI to enable communications
    start_twai_receive();
}