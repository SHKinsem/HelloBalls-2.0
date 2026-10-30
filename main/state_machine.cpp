#include "serial.h"
#include "cmd_hub.h"
#include "leds.h"
#include "app_motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "stepper_motors.h"

#define TAG "STATE_MACHINE"

SemaphoreHandle_t wheelControl_handle = NULL; // Handle for wheel control task  

bool isShot = false; // Flag to indicate if a shot has been made

// Delay inside an actuator sequence. Returns false if an emergency stop latched
// meanwhile, after making sure nothing keeps moving; the sequence must then end.
static bool sequence_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
    if (!cmd_hub_estopped()) {
        return true;
    }
    ESP_LOGW(TAG, "Emergency stop: sequence aborted");
    motorStateHelper(false);
    set_wheel_motors_speed(0, 0);
    set_friction_wheels_speed(0);
    stop_stepper_motor();
    disable_servos();
    return false;
}

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

    while (1) {
        bool changed = false;
        // Wait for state change signal from any command source (serial host or BLE remote)
        host_state_t host_state = (host_state_t)cmd_hub_wait_state_change(portMAX_DELAY, &changed);
        if (!changed) {
            continue;
        }
        // enable_servos(); // Ensure servos are enabled
        switch (host_state)
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
            tilt_servos(-20);
            if (!sequence_delay(1000)) break; // Wait for servos to tilt
            tilt_servos(-5); // Reset servos to neutral position
            // Update the MCU state to indicate searching for a ball
            *mcu_state = MCU_SEARCHING_BALL;
            update_led_state_noHandle(SEARCHING_BALL); // Update LED state
            break;
        
        case HOST_BALL_REACHED:
            ESP_LOGI(TAG, "HOST_BALL_REACHED state");
            enable_servos();
            set_friction_wheels_speed(-300);
            tilt_servos(30);
            if (!sequence_delay(1000)) break;
            disable_servos(); // Disable servos after tilting
            moveLoader(20.0f);
            if (!sequence_delay(500)) break;
            moveLoader(-110.0f);
            if (!sequence_delay(1000)) break;
            enable_servos();
            tilt_servos(-10);
            if (!sequence_delay(500)) break;
            tilt_servos(-3);
            if (!sequence_delay(500)) break;
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
            set_wheel_motors_speed(0, 0);
            if(isShot){
                home_stepper_motor();
                isShot = false;
            }
            set_stepper_pos(1600); // Reset stepper position
            if (!sequence_delay(2000)) break; // Wait for friction wheels to reach speed
            moveLoader(135.0f); // Move loader to shooting position
            if (!sequence_delay(1000)) break; // Wait for loader to move
            moveLoader(-45.0f);
            if (!sequence_delay(500)) break; // Wait for loader to move
            resetLoaderOrigin();
            isShot = true; // Set shot flag to true
            set_friction_wheels_speed(0); // Stop friction wheels
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


// Holds the task that monitors command activity and controls motors based on received commands
void serial_watchdog_task(void *arg) {
    ESP_LOGI(TAG, "Command watchdog task started");
    robot_cmd_t cmd = {};
    const robot_cmd_t* rx_msg = &cmd;
    
    const TickType_t xSerialTimeout = pdMS_TO_TICKS(500); // 500ms timeout
    
    while (1) {

        // Wait for a command from the source in control (serial host or BLE remote)
        if (!cmd_hub_wait_command(&cmd, xSerialTimeout)) {
            // Timeout occurred (or emergency stop latched), no command received
            motorStateHelper(false); // Ensure motors are off
            set_wheel_motors_speed(0, 0); // Stop wheel motors
            set_friction_wheels_speed(0); // Stop friction wheels
            disable_servos();
            continue; // Go back to waiting
        }

        motorStateHelper(true);
        // Process commands based on host state
        int16_t tilt_angle;
        switch((host_state_t)cmd_hub_host_state())
        {
        case HOST_SEARCHING_BALL:
            // Fall through to set wheel motors speed
        case HOST_BALL_REACHED:
            set_wheel_motors_speed(rx_msg->wheel1_speed, rx_msg->wheel2_speed);
            break;

        case HOST_SHOOTING:
            tilt_servos(rx_msg->tilt_angle); // Tilt servos based on received angle
            set_friction_wheels_speed(rx_msg->shoot_speed); // Set speed for friction wheels

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
    enable_servos();
    tilt_servos(0); // Initialize servos to neutral position
    vTaskDelay(pdMS_TO_TICKS(1000));
    disable_servos();
    xTaskCreate(state_machine_task, "state_machine_task", 4096, NULL, 5, NULL);
    xTaskCreate(serial_watchdog_task, "serial_watchdog_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "State machine task created");
}