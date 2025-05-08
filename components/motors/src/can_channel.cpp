#include "can_channel.h"
#include "esp_err.h"
#include "esp_log.h"
#include <cstring>

#define TAG "CAN_CHANNEL"


can_channel_t::can_channel_t(uint8_t channel_id, gpio_num_t tx_io, gpio_num_t rx_io):
    channel_id(channel_id), 
    motorCount(0), 
    motors(nullptr), 
    twai_running(false),
    tx_failed_count(0),
    last_rx_time(0),
    tx_active(false),
    rx_timeout_ms(500) {

    t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps

    f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    g_config.controller_id = 0,             
    g_config.mode = TWAI_MODE_NORMAL;
    g_config.tx_io = tx_io;
    g_config.rx_io = rx_io;       
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

void can_channel_t::reg_motor(base_motor_t* motor) {
    if (motorCount < 8 && motor->getMotorId() < 8) { // Assuming a maximum of 8 motors per channel
        motors[motor->getMotorId() - 1] = motor;   // Register the motor at the specified ID, else overwrite the existing one
        motor->setCanChannel(this); // Set the CAN channel for the motor
        motorCount++;
        ESP_LOGI(TAG, "Motor %d registered on channel %d", motor->getMotorId(), channel_id);
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
            // Check if we've exceeded the RX timeout and need to stop TX
            if (tx_active && (xTaskGetTickCount() - last_rx_time) > pdMS_TO_TICKS(rx_timeout_ms)) {
                tx_active = false;
                ESP_LOGI(TAG, "No RX messages for %lu ms, stopping TX on channel %d", rx_timeout_ms, channel_id);
            }
            continue;
        }

        // Update last reception time whenever we receive a valid message
        last_rx_time = xTaskGetTickCount();
        
        // If TX is not active, activate it now that we've received a message
        if (!tx_active) {
            tx_active = true;
            ESP_LOGI(TAG, "RX message received, enabling TX on channel %d", channel_id);
        }

        int msg_id = rx_msg.identifier % 0x200;
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
        // Skip if TWAI driver is not running
        if(!twai_running) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Skip transmission if tx_active is false (no recent RX messages)
        if(!tx_active) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Check more frequently than the main delay
            continue;
        }

        // Update motor control outputs
        updateMotorControlOutput(); // Update the control outputs for all motors

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
    
    while (true) {
        if (!twai_running) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Wait for alerts with a timeout
        esp_err_t res = twai_read_alerts_v2(twai_handle, &alerts, pdMS_TO_TICKS(1000));
        if (res == ESP_OK && alerts) {
            // Only attempt recovery for critical errors
            if (alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS)) {
                ESP_LOGW(TAG, "TWAI channel %d critical error detected, attempting recovery", channel_id);
                
                // Simple recovery: stop and restart the driver
                stop();
                vTaskDelay(pdMS_TO_TICKS(500));
                start();
                ESP_LOGI(TAG, "TWAI channel %d restarted after error", channel_id);
            }
            
            // Clear RX queue if full
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                ESP_LOGW(TAG, "TWAI Alert: Receive queue full - channel %d", channel_id);
                twai_message_t msg;
                int cleared = 0;
                while (twai_receive_v2(twai_handle, &msg, 0) == ESP_OK && cleared < 20) {
                    cleared++;
                }
                ESP_LOGI(TAG, "Cleared %d messages from RX queue", cleared);
            }
            
            // Handle TX failures with brief pause
            if (alerts & TWAI_ALERT_TX_FAILED) {
                static uint32_t tx_failed_count = 0;
                if (++tx_failed_count > 10) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    tx_failed_count = 0;
                }
            }
        } 
        else if (res != ESP_ERR_TIMEOUT && res != ESP_OK) {
            ESP_LOGE(TAG, "Error reading TWAI alerts: %s", esp_err_to_name(res));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    vTaskDelete(NULL);
}

bool can_channel_t::handle_twai_alert(uint32_t alerts) {
    bool recovery_needed = false;

    // Only check for the most critical errors that require recovery
    if (alerts & TWAI_ALERT_BUS_OFF) {
        ESP_LOGE(TAG, "TWAI Alert: Bus Off condition - channel %d", channel_id);
        recovery_needed = true;
    }
    
    if (alerts & TWAI_ALERT_ERR_PASS) {
        ESP_LOGW(TAG, "TWAI Alert: Error Passive state - channel %d", channel_id);
        recovery_needed = true;
    }
    
    // Report bus errors but don't trigger recovery
    if (alerts & TWAI_ALERT_BUS_ERROR) {
        ESP_LOGW(TAG, "TWAI Alert: Bus errors detected - channel %d", channel_id);
    }
    
    // Clear recovery needed flag if we've recovered
    if (alerts & (TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_ACTIVE)) {
        ESP_LOGI(TAG, "TWAI channel %d recovered from error state", channel_id);
        recovery_needed = false;
    }

    return recovery_needed;
}

bool can_channel_t::recover_from_bus_off() {
    ESP_LOGI(TAG, "Attempting to recover from Bus Off state - channel %d", channel_id);
    
    // Simple recovery by cycling the TWAI driver
    esp_err_t status = twai_stop_v2(twai_handle);
    if (status != ESP_OK) {
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    status = twai_start_v2(twai_handle);
    if (status != ESP_OK) {
        return false;
    }
    
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

void can_channel_t::updateMotorControlOutput(){
    // Update the control output for each motor in the channel
    for (int i = 0; i < 8; i++) {
        if (motors[i]) {
            motors[i]->calOutput();
        }
    }
}

void can_channel_t::sendMessage(twai_message_t* msg){
    twai_transmit_v2(twai_handle, msg, pdMS_TO_TICKS(5)); // Transmit the first message
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
    
    // Initialize our reception tracking variables
    last_rx_time = xTaskGetTickCount();
    tx_active = false;  // Start with TX inactive until we receive the first message
    
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