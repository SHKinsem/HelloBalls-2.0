/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a master node in a TWAI network. The master
 * node is responsible for initiating and stopping the transfer of data messages.
 * The example will execute multiple iterations, with each iteration the master
 * node will do the following:
 * 1) Start the TWAI driver
 * 2) Repeatedly send ping messages until a ping response from slave is received
 * 3) Send start command to slave and receive data messages from slave
 * 4) Send stop command to slave and wait for stop response from slave
 * 5) Stop the TWAI driver
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "esp32_s3_szp.h"
#include "app_ui.h"
#include "app_button.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define PING_PERIOD_MS          250
#define NO_OF_DATA_MSGS         10
#define NO_OF_ITERS             3
#define ITER_DELAY_MS           1000
#define RX_TASK_PRIO            8
#define TX_TASK_PRIO            9
#define CTRL_TSK_PRIO           10
#define TX_GPIO_NUM             11
#define RX_GPIO_NUM             10
#define EXAMPLE_TAG             "TWAI Master"

#define ID_SLAVE_DATA           0x0B1
#define ID_DJI_RM_MOTOR         0x200

typedef enum {
    TX_SEND_SPEED,
    TX_SEND_START_CMD,
    TX_SEND_STOP_CMD,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum {
    RX_RECEIVE_PING_RESP,
    RX_RECEIVE_DATA,
    RX_RECEIVE_STOP_RESP,
    RX_TASK_EXIT,
} rx_task_action_t;

// static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

static twai_message_t speed_message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = ID_DJI_RM_MOTOR,
    .data_length_code = 8,
    .data = {0},
};

// Add this function before using it
static void update_can_message(twai_message_t *msg, int16_t motor1_speed, int16_t motor2_speed, 
    int16_t motor3_speed, int16_t motor4_speed) {
    // For DJI motors, each motor uses 2 bytes in the message
    msg->data[0] = (motor1_speed >> 8) & 0xFF;    // High byte of motor 1
    msg->data[1] = motor1_speed & 0xFF;           // Low byte of motor 1

    msg->data[2] = (motor2_speed >> 8) & 0xFF;    // High byte of motor 2
    msg->data[3] = motor2_speed & 0xFF;           // Low byte of motor 2

    msg->data[4] = (motor3_speed >> 8) & 0xFF;    // High byte of motor 3
    msg->data[5] = motor3_speed & 0xFF;           // Low byte of motor 3

    msg->data[6] = (motor4_speed >> 8) & 0xFF;    // High byte of motor 4
    msg->data[7] = motor4_speed & 0xFF;           // Low byte of motor 4
}

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t stop_receive_sem;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t done_sem;


/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    while (1) {
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);

         if (action == RX_RECEIVE_DATA) {
            //Receive data messages from slave
            uint32_t data_msgs_rec = 0;
            ESP_LOGI(EXAMPLE_TAG, "Waiting for data messages from slave...");
            while (xSemaphoreTake(stop_receive_sem, 0) != pdTRUE) {
                twai_message_t rx_msg;
                esp_err_t status = twai_receive(&rx_msg, 1000 / portTICK_PERIOD_MS);
                if(status == ESP_ERR_TIMEOUT) {
                    ESP_LOGI(EXAMPLE_TAG, "Timeout waiting for data message from slave");
                    break;
                } else if (status != ESP_OK) {
                    ESP_LOGE(EXAMPLE_TAG, "Error receiving message: %s", esp_err_to_name(status));
                    break;
                }
                int msg_id = rx_msg.identifier % ID_DJI_RM_MOTOR;
                // if (rx_msg.identifier == ID_SLAVE_DATA) {
                if(msg_id > 0 && msg_id < 5) {
                    uint32_t data = 0;
                    for (int i = 0; i < rx_msg.data_length_code; i++) data |= (rx_msg.data[i] << (i * 8));
                    ESP_LOGI(EXAMPLE_TAG, "Received data value from id %d %"PRIu32, msg_id, data);
                    data_msgs_rec ++;
                } else {
                    ESP_LOGI(EXAMPLE_TAG, "Received unknown message with ID %d", msg_id);
                }

            }
        } else if (action == RX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    int16_t motor1_speed = 700;   // These could be variables from elsewhere
    int16_t motor2_speed = 0;   // in your program
    int16_t motor3_speed = 0;
    int16_t motor4_speed = 0;
    while (1) {
        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_SPEED) {
            //Repeatedly transmit motor speeds
            ESP_LOGI(EXAMPLE_TAG, "Transmitting motor speeds");
            uint32_t data_transmitted = 0;
            while (data_transmitted < 500) {
                // Update the message with current motor speed values
                update_can_message(&speed_message, motor1_speed, motor2_speed, 
                                  motor3_speed, motor4_speed);
                esp_err_t status = twai_transmit(&speed_message, pdMS_TO_TICKS(1000));
                if(status == ESP_ERR_TIMEOUT) {
                    ESP_LOGI(EXAMPLE_TAG, "Timeout waiting for twai_transmit");
                    break;
                } else if (status != ESP_OK) {
                    ESP_LOGE(EXAMPLE_TAG, "Error twai_transmit message: %s", esp_err_to_name(status));
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(5));
                data_transmitted ++;
            }
            xSemaphoreGive(stop_receive_sem); // Stop receiving data messages
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_SEND_STOP_CMD){
            ESP_LOGI(EXAMPLE_TAG, "Sending stop command to motors");
            while(motor1_speed > 0) {
                motor1_speed -= 10; // Gradually decrease speed to 0
                motor2_speed -= 10;
                motor3_speed -= 10;
                motor4_speed -= 10;
                vTaskDelay(pdMS_TO_TICKS(PING_PERIOD_MS));
                update_can_message(&speed_message, motor1_speed, motor2_speed, motor3_speed, motor4_speed);
                esp_err_t status = twai_transmit(&speed_message, pdMS_TO_TICKS(1000));
                if(status == ESP_ERR_TIMEOUT) {
                    ESP_LOGI(EXAMPLE_TAG, "Timeout waiting for twai_transmit");
                    break;
                } else if (status != ESP_OK) {
                    ESP_LOGE(EXAMPLE_TAG, "Error twai_transmit message: %s", esp_err_to_name(status));
                    break;
                }
            }
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action;
    rx_task_action_t rx_action;

    for (int iter = 0; iter < NO_OF_ITERS; iter++) {
        // xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(EXAMPLE_TAG, "Driver started");
        xSemaphoreGive(ctrl_task_sem);
        tx_action = TX_SEND_SPEED;
        rx_action = RX_RECEIVE_DATA;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        ESP_LOGI(EXAMPLE_TAG, "Waiting for semaphore");
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        // ESP_ERROR_CHECK(twai_stop());
        // ESP_ERROR_CHECK(twai_start());
        
        // tx_action = TX_SEND_STOP_CMD;
        // rx_action = RX_RECEIVE_DATA;
        // xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        // xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        // xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
        vTaskDelay(pdMS_TO_TICKS(ITER_DELAY_MS));
    }

    ESP_ERROR_CHECK(twai_start());
    tx_action = TX_SEND_STOP_CMD;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_stop());

    //Stop TX and RX tasks
    tx_action = TX_TASK_EXIT;
    rx_action = RX_TASK_EXIT;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    //Delete Control task
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}


void can_task(void){

    //Create tasks, queues, and semaphores
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    ctrl_task_sem = xSemaphoreCreateBinary();
    stop_receive_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);

    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    // uint32_t alerts = {TWAI_ALERT_TX_FAILED, TWAI_ALERT_RX_QUEUE_FULL};
    // twai_read_alerts(&alerts, pdMS_TO_TICKS(1000));

    xSemaphoreGive(ctrl_task_sem);              //Start control task
    xSemaphoreTake(done_sem, portMAX_DELAY);    //Wait for completion

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vQueueDelete(rx_task_queue);
    vQueueDelete(tx_task_queue);
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(stop_receive_sem);
    vSemaphoreDelete(done_sem);
}

void app_main(void)
{
    bsp_i2c_init();  // I2C initialization
    pca9557_init();  // IO expander initialization
    bsp_lvgl_start(); // Lvgl initialization
    bsp_codec_init(); // Codec initialization

    if(bsp_sdcard_mount() == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG, "SD card mounted successfully");
        mp3_player_init();
    } else {
        ESP_LOGE(EXAMPLE_TAG, "Failed to mount SD card");
    }

    ESP_LOGI(EXAMPLE_TAG, "TWAI Master Example");
    button_init(NULL); // Initialize button with no callback function
    // can_task();
    ESP_LOGI(EXAMPLE_TAG, "TWAI Master Example completed");
}