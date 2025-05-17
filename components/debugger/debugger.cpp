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

m3508_t frictionWheel_1(1);
m3508_t frictionWheel_2(2);
m3508_t wheelMotor_1(3);
m3508_t wheelMotor_2(4);



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

// #define RUN_MOTOR_PERIOD 10000
// #define MESSAGE_FREQUENCY 1 // Frequency of sending messages in ms
// #define MAX_COUNTER RUN_MOTOR_PERIOD / MESSAGE_FREQUENCY // Maximum counter value

// void debugLoggingTask(void *arg) {
//     while (1) {
//         // Only transmit zeros when button is not pressed, and no serial control is active
//         if (!*get_button_state_ptr()) {
//             // Skip sending zero messages - serialWheelControlTask handles this now
//             // twai_transmit_speed(0, 0, 0, 0); <- removed to avoid conflict
//         }
        
//         if(*get_button_state_ptr()) {

//             motor_ptr[0]->enable(); // Use the pointer to enable the motor
//             motor_ptr[1]->enable(); // Use the pointer to enable the motor
//             ESP_LOGI(TAG, "Motor started, output: %d", output);
//             lv_obj_set_style_bg_color(ui_MainMenuButtonTestButton, lv_color_hex(0xFF0085), LV_PART_MAIN | LV_STATE_DEFAULT);
//             TickType_t xLastWakeTime = xTaskGetTickCount(); // Get the current tick count

//             for (int counter = 0; counter < MAX_COUNTER; counter++) {
//                 if (!get_twai_running()) break;
//                 if(counter % 1000 == 0) ESP_LOGI(TAG, "Counter: %d", counter);
//                 twai_transmit_speed(motor_ptr[0]->calOutput(), motor_ptr[1]->calOutput(), 0, 0); // Transmit the speed to the motor
//                 // ESP_LOGI(TAG, "O: %d, R: %d, T: %d", output, motor_ptr[0]->getRawSpeed(), motor_ptr[0]->getTargetSpeed());
//                 xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MESSAGE_FREQUENCY)); // Delay until the next period
//                 // vTaskDelay(pdMS_TO_TICKS(MESSAGE_FREQUENCY)); // Delay for the message frequency
//             }
//             motor_ptr[0]->disable();
//             motor_ptr[1]->disable();
            
//             *get_button_state_ptr() = false; // Reset button state
//             lv_obj_set_style_bg_color(ui_MainMenuButtonTestButton, lv_color_hex(0x838383), LV_PART_MAIN | LV_STATE_DEFAULT);
//         }
//         // ESP_LOGI(TAG, "R: %d, T: %d", motor_ptr[0]->getRawSpeed(), motor_ptr[0]->getTargetSpeed());

//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

void motorStateHelper(bool state) {
    if (state) {
        frictionWheel_1.enable(); // Enable the motor
        frictionWheel_2.enable(); // Enable the motor
        wheelMotor_1.enable(); // Enable the motor
        wheelMotor_2.enable(); // Enable the motor
    } else {
        frictionWheel_1.disable();
        frictionWheel_2.disable();
        wheelMotor_1.disable();
        wheelMotor_2.disable();
    }
}

void serialWheelControlTask(void *arg) {
    ESP_LOGI(TAG, "Serial wheel control task started");
    // Setup for serial activity detection
    rx_message_t* rx_msg = get_rx_message_ptr();
    task_state_t* task_state = getTaskState(); // Get the task state pointer
    
    while (1) {
        if (*task_state == RECEIVING) { motorStateHelper(true); } // Enable the motor if receiving data
        else { motorStateHelper(false); } // Disable the motor if not receiving data

         // Get the machine state from the received message in advance
         // Avoid the state change during the task execution
        uint8_t machine_state = rx_msg->machine_state;
        wheelMotor_1.setTargetSpeed(rx_msg->wheel1_speed);
        wheelMotor_2.setTargetSpeed(-rx_msg->wheel2_speed);        
        if(machine_state == 0) {
            frictionWheel_1.setTargetSpeed(0);
            frictionWheel_2.setTargetSpeed(0);
        } else if (machine_state == 1) {
            frictionWheel_1.setTargetSpeed(1000);
            frictionWheel_2.setTargetSpeed(-1000);
        } else if (machine_state == 2) {
            frictionWheel_1.setTargetSpeed(-1000);
            frictionWheel_2.setTargetSpeed(1000);
        }
        vTaskDelay(pdMS_TO_TICKS(20));

    }
}

void motor_task_init(){
    can_channel.reg_motor(&test_motor); // Register the motor with the CAN channel
    can_channel.reg_motor(&frictionWheel_1);
    can_channel.reg_motor(&frictionWheel_2);
    can_channel.reg_motor(&wheelMotor_1);
    can_channel.reg_motor(&wheelMotor_2);

    frictionWheel_1.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters
    frictionWheel_2.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters
    wheelMotor_1.setPIDParameters(30.0, 0.02, 0.1, 0.1, 0.1, 2000.0, -2000.0); // Set PID parameters
    wheelMotor_2.setPIDParameters(30.0, 0.02, 0.1, 0.1, 0.1, 2000.0, -2000.0); // Set PID parameters
 
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms

    can_channel.start(); // Start the CAN channel
    // test_motor.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters
    // Create the new serial wheel control task

    xTaskCreatePinnedToCore(serialWheelControlTask, "Serial Wheel Control", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 0);
}