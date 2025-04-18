#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "bldc_motors.h"
#include "stepper_motors.h"
#include "rm2006.h"
#include "rm3508.h"
#include "dm3519.h"
// #include "ui.h"
#include "app_twai.h"
#include "app_motors.h"
#include "esp_task_wdt.h"

#define TAG "MOTORS"
#define ID_DJI_RM_MOTOR         0x200

rm3508_t frictionwheels[2] = {rm3508_t(1), rm3508_t(2)}; // Create instances of rm3508 motors for friction wheels

// static int stepper_motor_pos = 0; // Current position of the stepper motor

base_motor_t* get_motor_ptr(uint8_t motor_id) {
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
        if(msg_id > 0 && msg_id < 5) {
            // for (int i = 0; i < rx_msg.data_length_code; i++) motor_data[msg_id-1][i] = rx_msg.data[i];
            frictionwheels[msg_id - 1].parseData(rx_msg.data); // Parse data for the specific motor
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