#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <cstring>  // For strcat function
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

m3508_t frictionwheels[2] = {m3508_t(1), m3508_t(2)}; // Create instances of m3508 motors for friction wheels

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

can_channel_t::can_channel_t(uint8_t channel_id):
    channel_id(channel_id), 
    motorCount(0), 
    motors(nullptr), 
    twai_running(false) {

    t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps

    f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    g_config.controller_id = 0,             
    g_config.mode = TWAI_MODE_NORMAL;
    g_config.tx_io = TWAI_TX_PIN;
    g_config.rx_io = TWAI_RX_PIN;       
    g_config.clkout_io = TWAI_IO_UNUSED; 
    g_config.bus_off_io = TWAI_IO_UNUSED;  
    g_config.tx_queue_len = 5;
    g_config.rx_queue_len = 5;                          
    g_config.alerts_enabled = TWAI_ALERT_ALL;
    g_config.clkout_divider = 0;
    g_config.intr_flags = ESP_INTR_FLAG_LEVEL2;

    twai_driver_install_v2(&g_config, &t_config, &f_config, &twai_handle); // Install the TWAI driver

    for(int i = 0; i < 2; i++) {
        tx_message[i].extd = 0; // Standard Format message (11-bit ID)
        tx_message[i].rtr = 0;  // Send a data frame
        tx_message[i].ss = 1;   // Is single shot (won't retry on error or NACK)
        tx_message[i].self = 0; // Not a self reception request
        tx_message[i].dlc_non_comp = 0; // DLC is less than 8
        tx_message[i].data_length_code = 8;
        for(int j = 0; j < 8; j++) {
            tx_message[i].data[j] = 0;
        }
    }

    tx_message[0].identifier = 0x200;
    tx_message[1].identifier = 0x1FF;

}

can_channel_t::~can_channel_t() {
    stop(); // Make sure to stop all tasks and the TWAI driver
}

void can_channel_t::reg_motor(base_motor_t* motor, uint8_t motor_id) {
    if (motorCount < 8 && motor_id < 8) { // Assuming a maximum of 8 motors per channel
        motors[motor_id] = motor;   // Register the motor at the specified ID, else overwrite the existing one
        motorCount++;
    } else {
        ESP_LOGE(TAG, "Reg failed");
    }
}

void can_channel_t::rx_task(void* arg) {
    while (true) {
        static twai_message_t rx_msg;

        if(!twai_running) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        esp_err_t status = twai_receive_v2(twai_handle, &rx_msg, pdMS_TO_TICKS(1000));
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "Error receiving message: %s", esp_err_to_name(status));
            continue;
        }

        int msg_id = rx_msg.identifier % ID_DJI_RM_MOTOR;
        if(motors[msg_id - 1]) {
            motors[msg_id - 1]->parseData(rx_msg.data);
        } else {
            ESP_LOGI(TAG, "Unregistered motor msg with ID %d", msg_id);
        }
    }
    vTaskDelete(NULL);
}

void can_channel_t::tx_task(void* arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Get the current tick count

    while (true) {

        if(!twai_running) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        for(int i = 0; i < 8; i++) {
            int msg_index = i / 4;     // 0 for motors 0-3, 1 for motors 4-7
            int data_index = i % 4;    // Position within the message (0-3)
            
            if(motors[i]) {
                tx_message[msg_index].data[data_index * 2] = (motors[i]->getControlOutput() >> 8) & 0xFF; // High byte
                tx_message[msg_index].data[data_index * 2 + 1] = motors[i]->getControlOutput() & 0xFF;    // Low byte
            } else {
                tx_message[msg_index].data[data_index * 2] = 0;     // Set to 0 if motor is not registered
                tx_message[msg_index].data[data_index * 2 + 1] = 0;
            }
        }

        twai_transmit_v2(twai_handle, &tx_message[0], pdMS_TO_TICKS(5)); // Transmit the first message
        twai_transmit_v2(twai_handle, &tx_message[1], pdMS_TO_TICKS(5)); // Transmit the second message
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1)); // Precise delay for 1ms
    }

    vTaskDelete(NULL);
}

void can_channel_t::alert_task(void* arg) {
    uint32_t alerts;
    bool recovery_needed = false;
    
    while (true) {
        if (!twai_running) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Wait for alerts with a timeout
        esp_err_t res = twai_read_alerts_v2(twai_handle, &alerts, pdMS_TO_TICKS(1000));
        if (res == ESP_OK) {
            // Log and handle alerts
            log_alert_flags(alerts);
            recovery_needed = handle_twai_alert(alerts);
            
            if (recovery_needed) {
                ESP_LOGW(TAG, "TWAI channel %d requires recovery, attempting...", channel_id);
                // If recovery failed, we might need to stop and restart the driver
                if (!recover_from_bus_off() && !recover_from_error_passive()) {
                    ESP_LOGE(TAG, "TWAI channel %d recovery failed, restarting driver", channel_id);
                    stop();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    start();
                }
            }
        } else if (res != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Error reading TWAI alerts: %s", esp_err_to_name(res));
            vTaskDelay(pdMS_TO_TICKS(100)); // Avoid tight loop on error
        }
    }
    
    vTaskDelete(NULL);
}

bool can_channel_t::handle_twai_alert(uint32_t alerts) {
    bool recovery_needed = false;

    // Handle bus off condition - most serious error
    if (alerts & TWAI_ALERT_BUS_OFF) {
        ESP_LOGE(TAG, "TWAI Alert: Bus Off condition - channel %d", channel_id);
        recovery_needed = true;
    }
    
    // Handle error passive condition
    if (alerts & TWAI_ALERT_ERR_PASS) {
        ESP_LOGW(TAG, "TWAI Alert: Error Passive state - channel %d", channel_id);
        recovery_needed = true;
    }
    
    // Handle bus error condition (CAN errors detected)
    if (alerts & TWAI_ALERT_BUS_ERROR) {
        ESP_LOGW(TAG, "TWAI Alert: Bus errors detected - channel %d", channel_id);
        // Count is incremented by ISR already
    }

    // Handle above error warning limit (TEC or REC exceeded warning threshold)
    if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
        ESP_LOGW(TAG, "TWAI Alert: Error counter above warning limit - channel %d", channel_id);
        // This is a warning that error counters are increasing
    }

    // Handle receive queue full condition
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
        ESP_LOGW(TAG, "TWAI Alert: Receive queue full - channel %d", channel_id);
        recover_from_rx_queue_full();
    }
    
    // Handle RX FIFO overrun condition
    if (alerts & TWAI_ALERT_RX_FIFO_OVERRUN) {
        ESP_LOGW(TAG, "TWAI Alert: RX FIFO overrun - channel %d", channel_id);
        // This is handled internally by the TWAI driver already
    }
    
    // Handle TX failed condition
    if (alerts & TWAI_ALERT_TX_FAILED) {
        ESP_LOGW(TAG, "TWAI Alert: TX Failed - channel %d", channel_id);
        // If TX keeps failing, it might be worth initiating a brief transmit pause
        if (++tx_failed_count > 10) { // Threshold to consider action
            vTaskDelay(pdMS_TO_TICKS(50)); // Brief pause in transmission
            tx_failed_count = 0;
            ESP_LOGI(TAG, "TX failures threshold reached, brief transmit pause initiated");
        }
    }

    // Handle arbitration lost condition
    if (alerts & TWAI_ALERT_ARB_LOST) {
        ESP_LOGW(TAG, "TWAI Alert: Arbitration lost - channel %d", channel_id);
        // Normal in multi-master systems, just information
    }

    // Handle recovery completion
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        ESP_LOGI(TAG, "TWAI Alert: Bus recovered successfully - channel %d", channel_id);
        // Bus has recovered, we can clear any recovery flags
        recovery_needed = false;
    }
    
    // Handle recovery in progress notification
    if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
        ESP_LOGI(TAG, "TWAI Alert: Recovery in progress - channel %d", channel_id);
        // Recovery is already happening, no additional action needed
    }

    // Return to error active state (normal operation)
    if (alerts & TWAI_ALERT_ERR_ACTIVE) {
        ESP_LOGI(TAG, "TWAI Alert: Returned to error active state - channel %d", channel_id);
        // Error counters have decreased below thresholds, normal operation resumed
        recovery_needed = false;
    }

    // Below error warning limit notification
    if (alerts & TWAI_ALERT_BELOW_ERR_WARN) {
        ESP_LOGI(TAG, "TWAI Alert: Error counters below warning limit - channel %d", channel_id);
        // Error counters are at a good level
    }

    // Peripheral reset notification
    if (alerts & TWAI_ALERT_PERIPH_RESET) {
        ESP_LOGW(TAG, "TWAI Alert: Peripheral reset occurred - channel %d", channel_id);
        // The peripheral was reset automatically due to some error condition
    }

    return recovery_needed;
}

bool can_channel_t::recover_from_bus_off() {
    ESP_LOGI(TAG, "Attempting to recover from Bus Off state - channel %d", channel_id);
    
    // According to CAN specification, a node in Bus Off state must:
    // 1. Stop transmitting
    // 2. Wait for 128 occurrences of 11 consecutive recessive bits
    
    // ESP-IDF handles this automatically in the driver, but we need to:
    // 1. Stop current activity
    // 2. Reset the controller
    // 3. Restart the driver
    
    // First stop the driver
    esp_err_t status = twai_stop_v2(twai_handle);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop TWAI driver during recovery: %s", esp_err_to_name(status));
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow some time for the bus to stabilize
    
    // Restart the driver
    status = twai_start_v2(twai_handle);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart TWAI driver during recovery: %s", esp_err_to_name(status));
        return false;
    }
    
    ESP_LOGI(TAG, "TWAI channel %d recovered from Bus Off state", channel_id);
    return true;
}

bool can_channel_t::recover_from_error_passive() {
    ESP_LOGI(TAG, "Attempting to recover from Error Passive state - channel %d", channel_id);
    
    // In Error Passive state, a node can still communicate but has detected
    // many errors. Recovery strategy:
    // 1. Reduce transmission rate temporarily
    // 2. Monitor if we return to Error Active state
    
    // We can implement a simple recovery by suspending transmissions briefly
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay to let the bus recover
    
    // Check if we're still in error passive state
    uint32_t alerts;
    esp_err_t res = twai_read_alerts_v2(twai_handle, &alerts, 0);
    if (res == ESP_OK && (alerts & TWAI_ALERT_ERR_PASS)) {
        ESP_LOGW(TAG, "Still in Error Passive state after recovery attempt - channel %d", channel_id);
        return false;
    }
    
    ESP_LOGI(TAG, "TWAI channel %d recovered from Error Passive state", channel_id);
    return true;
}

bool can_channel_t::recover_from_rx_queue_full() {
    ESP_LOGI(TAG, "Clearing receive queue due to overflow - channel %d", channel_id);
    
    // When RX queue is full, we need to read and discard messages to clear it
    twai_message_t msg;
    int cleared_msgs = 0;
    esp_err_t res;
    
    // Read and discard messages until queue is no longer full
    do {
        res = twai_receive_v2(twai_handle, &msg, 0); // Non-blocking receive
        if (res == ESP_OK) {
            cleared_msgs++;
        }
    } while (res == ESP_OK && cleared_msgs < 20); // Limit to prevent infinite loop
    
    ESP_LOGI(TAG, "Cleared %d messages from RX queue - channel %d", cleared_msgs, channel_id);
    return (cleared_msgs > 0);
}

void can_channel_t::log_alert_flags(uint32_t flags) {
    // Only log if there are alerts present
    if (flags == 0) {
        return;
    }
    
    // Create a string representation of all active alert flags
    char alert_str[256] = {0};
    
    if (flags & TWAI_ALERT_TX_IDLE) strcat(alert_str, "TX_IDLE ");
    if (flags & TWAI_ALERT_TX_SUCCESS) strcat(alert_str, "TX_SUCCESS ");
    if (flags & TWAI_ALERT_TX_FAILED) strcat(alert_str, "TX_FAILED ");
    if (flags & TWAI_ALERT_RX_DATA) strcat(alert_str, "RX_DATA ");
    if (flags & TWAI_ALERT_BELOW_ERR_WARN) strcat(alert_str, "BELOW_ERR_WARN ");
    if (flags & TWAI_ALERT_ERR_ACTIVE) strcat(alert_str, "ERR_ACTIVE ");
    if (flags & TWAI_ALERT_RECOVERY_IN_PROGRESS) strcat(alert_str, "RECOVERY_IN_PROGRESS ");
    if (flags & TWAI_ALERT_BUS_RECOVERED) strcat(alert_str, "BUS_RECOVERED ");
    if (flags & TWAI_ALERT_ARB_LOST) strcat(alert_str, "ARB_LOST ");
    if (flags & TWAI_ALERT_ABOVE_ERR_WARN) strcat(alert_str, "ABOVE_ERR_WARN ");
    if (flags & TWAI_ALERT_BUS_ERROR) strcat(alert_str, "BUS_ERROR ");
    if (flags & TWAI_ALERT_RX_QUEUE_FULL) strcat(alert_str, "RX_QUEUE_FULL ");
    if (flags & TWAI_ALERT_ERR_PASS) strcat(alert_str, "ERR_PASS ");
    if (flags & TWAI_ALERT_BUS_OFF) strcat(alert_str, "BUS_OFF ");
    if (flags & TWAI_ALERT_RX_FIFO_OVERRUN) strcat(alert_str, "RX_FIFO_OVERRUN ");
    if (flags & TWAI_ALERT_PERIPH_RESET) strcat(alert_str, "PERIPH_RESET ");
    
    ESP_LOGD(TAG, "TWAI channel %d alerts: %s", channel_id, alert_str);
}

void can_channel_t::start() {
    // Install TWAI driver if not already installed
    esp_err_t status = twai_start_v2(twai_handle);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(status));
        return;
    }
    
    // Set flag to true before starting tasks
    twai_running = true;
    
    // Create RX task
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<can_channel_t*>(arg)->rx_task(arg); }, 
        "TWAI_rx_task", 
        4096,           // Stack size
        this,           // Pass 'this' pointer as argument
        5,              // Priority (higher than tx_task)
        NULL,           // Task handle not needed
        tskNO_AFFINITY  // Run on any available core
    );
    
    // Create TX task
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<can_channel_t*>(arg)->tx_task(arg); }, 
        "TWAI_tx_task", 
        4096,           // Stack size
        this,           // Pass 'this' pointer as argument
        4,              // Priority (lower than rx_task)
        NULL,           // Task handle not needed
        tskNO_AFFINITY  // Run on any available core
    );

    // Create Alert task
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<can_channel_t*>(arg)->alert_task(arg); }, 
        "TWAI_alert_task", 
        4096,           // Stack size
        this,           // Pass 'this' pointer as argument
        6,              // Priority (higher than rx_task and tx_task)
        NULL,           // Task handle not needed
        tskNO_AFFINITY  // Run on any available core
    );
    
    ESP_LOGI(TAG, "TWAI channel %d started successfully", channel_id);
}

void can_channel_t::stop() {
    // Set flag to false to signal tasks to stop
    twai_running = false;
    
    // Stop TWAI driver
    esp_err_t status = twai_stop_v2(twai_handle);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop TWAI driver: %s", esp_err_to_name(status));
    } else {
        ESP_LOGI(TAG, "TWAI channel %d stopped successfully", channel_id);
    }
    
    // Note: Tasks will self-terminate when they detect twai_running is false
    // We don't need to delete them explicitly as they check the flag
}
