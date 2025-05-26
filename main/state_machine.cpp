#include "serial.h"
#include "leds.h"
#include "app_motors.h"

#define TAG "STATE_MACHINE"

void state_machine_task(void *arg) {
    ESP_LOGI(TAG, "State machine task started");
    
    // Get the task state pointer
    serial_state_t* task_state = getTaskState();
    host_state_t* host_state = getHostState();
    mcu_state_t* mcu_state = getMcuState();
    
    while (1) {
        switch (*host_state)
        {
        case HOST_IDLE:
            ESP_LOGI(TAG, "HOST_IDLE state");
            break;

        case HOST_SEARCHING_BALL:
            ESP_LOGI(TAG, "HOST_SEARCHING_BALL state");
            // Update the MCU state to indicate searching for a ball
            *mcu_state = MCU_SEARCHING_BALL;
            update_led_state_noHandle(SEARCHING_BALL); // Update LED state
            break;
        
        case HOST_BALL_REACHED:
            ESP_LOGI(TAG, "HOST_BALL_REACHED state");
            // Update the MCU state to indicate ball has been reached
            *mcu_state = MCU_BALL_RETRIEVED;
            break;

        case HOST_SHOOTING:
            ESP_LOGI(TAG, "HOST_SHOOTING state");
            // Update the MCU state to indicate shooting
            *mcu_state = MCU_SHOOTING;
            // update_led_state_noHandle(SHOOTING); // Update LED state
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
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms before checking again
    }
}