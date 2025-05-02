#include "app_twai.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define PING_PERIOD_MS          250
#define SEND_DELAY_MS           1
#define NO_OF_ITERS             10
#define ITER_DELAY_MS           10
#define RX_TASK_PRIO            8
#define TX_TASK_PRIO            9
#define CTRL_TSK_PRIO           10
#define TWAI_TAG                "APP_TWAI"

#define ID_SLAVE_DATA           0x0B1


int16_t myGlobalSpeed = 0;
bool twai_running = false;

// static int getGlobalSpeed() {
//     return myGlobalSpeed;
// }

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
static const twai_general_config_t g_config = { .controller_id = 0,             
                                                .mode = TWAI_MODE_NO_ACK,
                                                // .mode = TWAI_MODE_NORMAL, 
                                                .tx_io = TWAI_TX_PIN, 
                                                .rx_io = TWAI_RX_PIN,        
                                                .clkout_io = TWAI_IO_UNUSED, 
                                                .bus_off_io = TWAI_IO_UNUSED,      
                                                .tx_queue_len = 5, 
                                                .rx_queue_len = 32,                           
                                                .alerts_enabled = TWAI_ALERT_ALL,  
                                                .clkout_divider = 0,        
                                                .intr_flags = ESP_INTR_FLAG_LEVEL2};

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
static SemaphoreHandle_t stop_transmit_sem;

static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t done_sem;


/* --------------------------- Tasks and Functions -------------------------- */


void twai_transmit_speed(int16_t speed1, int16_t speed2, int16_t speed3, int16_t speed4) {
    // Convert arg to int16_t
    if(speed1 == 0){
        speed_message.data[0] = 0;
        speed_message.data[1] = 0;
        speed_message.data[2] = 0;
        speed_message.data[3] = 0;
        speed_message.data[4] = 0;
        speed_message.data[5] = 0; 
        speed_message.data[6] = 0;
        speed_message.data[7] = 0;
    }
    else{
        speed_message.data[0] = (speed1 >> 8) & 0xFF;    // High byte of motor 1
        speed_message.data[1] = speed1 & 0xFF;           // Low byte of motor 1
        speed_message.data[2] = (speed2 >> 8) & 0xFF;    // High byte of motor 2
        speed_message.data[3] = speed2 & 0xFF;           // Low byte of motor 2
        speed_message.data[4] = (speed3 >> 8) & 0xFF;    // High byte of motor 3
        speed_message.data[5] = speed3 & 0xFF;           // Low byte of motor 3
        speed_message.data[6] = (speed4 >> 8) & 0xFF;    // High byte of motor 4
        speed_message.data[7] = speed4 & 0xFF;           // Low byte of motor 4
    }
    twai_transmit(&speed_message, pdMS_TO_TICKS(10));
}

void install_twai_driver(){
    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TWAI_TAG, "Driver installed");
}
void uninstall_twai_driver(){
    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(TWAI_TAG, "Driver uninstalled");
}

static uint8_t motor_data[4][8] = {0};

uint8_t* motorDataHook(uint8_t motor_id){
    return motor_data[motor_id - 1]; // Return the data for the requested motor ID
}

void twai_receive_task_continuous(void *arg)
{
    while (true) {
        if(!twai_running) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            // ESP_LOGI(TWAI_TAG, "TWAI driver not running, waiting...");
            continue;
        }

        //Receive data messages from slave
        twai_message_t rx_msg;
        esp_err_t status = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
        if (status != ESP_OK) {
            ESP_LOGE(TWAI_TAG, "Error receiving message: %s", esp_err_to_name(status));
            continue;
        }

        int msg_id = rx_msg.identifier % ID_DJI_RM_MOTOR;
        if(msg_id > 0 && msg_id < 5) {
            for (int i = 0; i < rx_msg.data_length_code; i++) motor_data[msg_id-1][i] = rx_msg.data[i];
            myGlobalSpeed = (uint16_t)((rx_msg.data[2] << 8) | rx_msg.data[3]); // Combine high and low bytes
        } else {
            ESP_LOGI(TWAI_TAG, "Received unknown message with ID %d", msg_id);
        }
        // vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TWAI_TAG, "TWAI receive task self-deleting...");
    vTaskDelete(NULL);
}

void start_twai_receive_task(void) {
    // Create a task to receive messages continuously
    xTaskCreate(twai_receive_task_continuous, "TWAI_rx_continuous", 4096, NULL, RX_TASK_PRIO, NULL);
}


void start_twai_receive(){
    ESP_LOGI(TWAI_TAG, "Starting to receive messages...");
    twai_running = true;
}

void stop_twai_receive(){
    ESP_LOGI(TWAI_TAG, "Stopping to receive messages...");
    twai_running = false;
}

bool get_twai_running(){
    return twai_running;
}