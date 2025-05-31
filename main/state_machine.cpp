#include "serial.h"
#include "leds.h"
#include "app_motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "stepper_motors.h"

#define TAG "STATE_MACHINE"

SemaphoreHandle_t wheelControl_handle = NULL; // Handle for wheel control task  

bool isShot = false; // Flag to indicate if a shot has been made

// Holds the tasks that execute only once when the state changes
void state_machine_task(void *arg) {
    ESP_LOGI(TAG, "State machine task started");

    wheelControl_handle = xSemaphoreCreateBinary(); // Create the semaphore
    if (wheelControl_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create wheel control semaphore");
        vTaskDelete(NULL);
    }

    // Get the task state pointer
    mcu_state_t* mcu_state = getMcuState();
    const host_state_t* host_state = getHostState();
    const rx_message_t* rx_msg = getRXmsg();

    while (1) {
        xSemaphoreTake(state_change_semaphore, portMAX_DELAY); // Wait for state change signal
        // enable_servos(); // Ensure servos are enabled
        switch (*host_state)
        {
        case HOST_IDLE:
            ESP_LOGI(TAG, "HOST_IDLE state");
            set_friction_wheels_speed(0); // Stop friction wheels
            disable_servos(); // Disable servos after tilting
            break;

        case HOST_SEARCHING_BALL:
            ESP_LOGI(TAG, "HOST_SEARCHING_BALL state");
            enable_servos();
            set_friction_wheels_speed(-800); // Set speed for friction wheels
            if(isShot){ 
                home_stepper_motor();
                isShot = false; // Reset shot flag after homing
            }
            set_stepper_pos(1150); // Reset stepper position
            tilt_servos(-10);
            vTaskDelay(pdMS_TO_TICKS(50)); // Wait for servos to tilt
            tilt_servos(-3); // Reset servos to neutral position
            // Update the MCU state to indicate searching for a ball
            *mcu_state = MCU_SEARCHING_BALL;
            update_led_state_noHandle(SEARCHING_BALL); // Update LED state
            break;
        
        case HOST_BALL_REACHED:
            ESP_LOGI(TAG, "HOST_BALL_REACHED state");
            enable_servos();
            set_friction_wheels_speed(-300);
            tilt_servos(30);
            vTaskDelay(pdMS_TO_TICKS(500));
            disable_servos(); // Disable servos after tilting
            moveLoader(20.0f);
            vTaskDelay(pdMS_TO_TICKS(500));
            moveLoader(-110.0f);
            vTaskDelay(pdMS_TO_TICKS(1000));
            enable_servos();
            tilt_servos(-10);
            tilt_servos(-3);
            vTaskDelay(pdMS_TO_TICKS(200));
            disable_servos(); // Disable servos after reaching the ball
            set_friction_wheels_speed(0); // Stop friction wheels
            resetLoaderOrigin();
            *mcu_state = MCU_BALL_RETRIEVED;
            break;

        case HOST_SHOOTING:
            ESP_LOGI(TAG, "HOST_SHOOTING state");
            // Update the MCU state to indicate shooting
            *mcu_state = MCU_SHOOTING;
            enable_servos();
            set_friction_wheels_speed(6500); // Set speed for friction wheels
            set_wheel_motors_speed(0, 0);
            if(isShot){
                home_stepper_motor();
                isShot = false;
            }
            set_stepper_pos(1600); // Reset stepper position
            vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for friction wheels to reach speed
            moveLoader(180.0f); // Move loader to shooting position
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for loader to move
            moveLoader(-90.0f);
            vTaskDelay(pdMS_TO_TICKS(500)); // Wait for loader to move
            resetLoaderOrigin();
            isShot = true; // Set shot flag to true
            // set_friction_wheels_speed(0); // Stop friction wheels
            break;
        case HOST_SCANNING:
            ESP_LOGI(TAG, "HOST_SCANNING state");
            enable_servos();
            // Update the MCU state to indicate scanning
            set_friction_wheels_speed(0); // Stop friction wheels
            break;

        case HOST_ERROR:
            ESP_LOGE(TAG, "HOST_ERROR state");
            // Update the MCU state to indicate an error
            *mcu_state = MCU_ERROR;
            update_led_state_noHandle(ERROR); // Update LED state
            break;

        default:
            break;
        }
    }
}


// Holds the task that monitors serial activity and controls motors based on received commands
void serial_watchdog_task(void *arg) {
    ESP_LOGI(TAG, "Serial watchdog task started");
    // Setup for serial activity detection
    const rx_message_t* rx_msg = getRXmsg();
    const serial_state_t* serial_state = getTaskState(); // Get the task state pointer
    const host_state_t* host_state = getHostState();
    
    const TickType_t xSerialTimeout = pdMS_TO_TICKS(500); // 500ms timeout
    
    while (1) {

        // Wait for the semaphore with timeout 
        if ((xSemaphoreTake(serial_rx_semaphore, xSerialTimeout) == pdFALSE) 
            || (*serial_state != SERIAL_RECEIVING)) {
            // Timeout occurred, no serial data received
            motorStateHelper(false); // Ensure motors are off
            set_wheel_motors_speed(0, 0); // Stop wheel motors
            set_friction_wheels_speed(0); // Stop friction wheels
            disable_servos();
            continue; // Go back to waiting
        }

        motorStateHelper(true);
        // Process commands based on host state
        int16_t tilt_angle;
        switch(*host_state)
        {
        case HOST_SEARCHING_BALL:
            // Fall through to set wheel motors speed
        case HOST_BALL_REACHED:
            set_wheel_motors_speed(rx_msg->wheel1_speed, rx_msg->wheel2_speed);
            break;

        case HOST_SHOOTING:
            tilt_servos(rx_msg->tilt_angle); // Tilt servos based on received angle
            break;

        case HOST_SCANNING:
            tilt_angle = rx_msg->tilt_angle;

            if (tilt_angle < -5){
                tilt_angle = -5;
            } else if (tilt_angle > 35) {
                tilt_angle = 35;
            }

            tilt_servos(tilt_angle);
            set_wheel_motors_speed(rx_msg->wheel1_speed, rx_msg->wheel2_speed);
            break;

        case HOST_ERROR:
            ESP_LOGE(TAG, "HOST_ERROR state in serial task");
            // Ensure motors are off in error state
            motorStateHelper(false);
            set_wheel_motors_speed(0, 0);
            set_friction_wheels_speed(0);
            break;

        default:
            // For any unhandled state, ensure motors are off
            motorStateHelper(false);
            set_wheel_motors_speed(0, 0);
            set_friction_wheels_speed(0);
            break;
        }
    }
}

void state_machine_init(void) {
    // Create the state machine task
    xTaskCreate(state_machine_task, "state_machine_task", 4096, NULL, 5, NULL);
    xTaskCreate(serial_watchdog_task, "serial_watchdog_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "State machine task created");
}