#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <cstring>
#include "bldc_motors.h"
#include "stepper_motors.h"
#include "m2006.h"
#include "m3508.h"
#include "dm3519.h"
// #include "ui.h"
#include "app_twai.h"
#include "app_motors.h"
#include "esp_task_wdt.h"

#define TAG "MOTORS"
#define ID_DJI_RM_MOTOR         0x200

static twai_message_t speed_message;

// Initialize the message
void init_speed_message() {
    // Message type and format settings
    speed_message.flags = TWAI_MSG_FLAG_SS;  // Single shot mode
    // Message ID and payload
    speed_message.identifier = ID_DJI_RM_MOTOR;
    speed_message.data_length_code = 8;
    memset(speed_message.data, 0, 8);  // Initialize all data bytes to 0
}

m3508_t frictionwheels[2] = {m3508_t(1), m3508_t(2)}; // Create instances of m3508 motors for friction wheels
m2006_t loaderMotor(3); // Create an instance of m2006 motor for loader motor

base_motor_t* get_motor_ptr(uint8_t motor_id) {
    if (motor_id == 3) return &loaderMotor;
    return &frictionwheels[motor_id - 1]; 
}

uint8_t* motor_data[4]; // Buffer to store motor data

void twai_receive_task_continuous(void *arg)
{
    twai_message_t rx_msg;

    while (true) {
        if(!get_twai_running()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            // ESP_LOGI(TAG, "TWAI driver not running, waiting...");
            continue;
        }

        //Receive data messages from slave
        esp_err_t status = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "Error receiving message: %s", esp_err_to_name(status));
            continue;
        }
        int msg_id = rx_msg.identifier % ID_DJI_RM_MOTOR;
        if(msg_id > 0 && msg_id < 3) {
            // for (int i = 0; i < rx_msg.data_length_code; i++) motor_data[msg_id-1][i] = rx_msg.data[i];
            frictionwheels[msg_id - 1].parseData(rx_msg.data); // Parse data for the specific motor
        } else if(msg_id == 3) {
            loaderMotor.parseData(rx_msg.data); // Parse data for the loader motor
        } else if(msg_id == 4) {
            // Handle DM3519 motor data here if needed
            // dm3519_motor.parseData(rx_msg.data); // Example for parsing DM3519 motor data
        } else {
            ESP_LOGI(TAG, "Received unknown message with ID %d", msg_id);
        }
        // esp_task_wdt_reset(); // Reset the watchdog timer
    }
    ESP_LOGI(TAG, "TWAI receive task self-deleting...");
    vTaskDelete(NULL);
}


void twai_init(){
    twai_start();
    stop_twai_receive();    // Stop receiving messages by default
    xTaskCreatePinnedToCore(twai_receive_task_continuous, "TWAI_rx_continuous", 4096, NULL, 1, NULL, tskNO_AFFINITY);
}