#include "serial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

/*
    RX message format:
    [machine_state, wheel1_speed, wheel2_speed]
    Frequency: 50 Hz

    TX message format:
    [machine_state, wheel1_distance, wheel2_distance, imu_x, imu_y, imu_z, imu_yaw]
    Frequency: 50 Hz
*/

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

static task_state_t task_state = IDLE; // Initialize task state to IDLE



// Global variables for message data
static rx_message_t rx_msg = {0, 0, 0};
static tx_message_t tx_msg = {0, 0, 0, 0.0, 0.0, 0.0, 0.0};

// Function to get pointer to RX data for external access
rx_message_t* get_rx_message(void) {
    return &rx_msg;
}

task_state_t* getTaskState(void) {
    return &task_state;
}

// Function to set TX data from external source
void set_tx_message(uint8_t state, int32_t w1_dist, int32_t w2_dist, 
                    float x, float y, float z, float yaw) {
    tx_msg.machine_state = state;
    tx_msg.wheel1_distance = w1_dist;
    tx_msg.wheel2_distance = w2_dist;
    tx_msg.imu_x = x;
    tx_msg.imu_y = y;
    tx_msg.imu_z = z;
    tx_msg.imu_yaw = yaw;
}

// Initialize UART communication
static void uart_init_port(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}

// Send data through UART
int sendUartData(const char* logName, const uint8_t* data, size_t len)
{
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    ESP_LOGI(logName, "Wrote %d bytes to UART", txBytes);
    return txBytes;
}

// Send data through all available interfaces
int sendData(const char* logName, const uint8_t* data, size_t len)
{
    return sendUartData(logName, data, len);
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    
    // Buffer for serializing tx_msg
    uint8_t tx_buffer[32]; // Large enough for our message
    
    while (1) {
        if (task_state == IDLE) {
            vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz = 20ms period
            continue;
        }
        
        // Serialize tx_msg into tx_buffer
        tx_buffer[0] = tx_msg.machine_state;
        
        // Wheel 1 distance (4 bytes)
        memcpy(&tx_buffer[1], &tx_msg.wheel1_distance, sizeof(int32_t));
        
        // Wheel 2 distance (4 bytes)
        memcpy(&tx_buffer[5], &tx_msg.wheel2_distance, sizeof(int32_t));
        
        // IMU data (4 floats, 16 bytes)
        memcpy(&tx_buffer[9], &tx_msg.imu_x, sizeof(float));
        memcpy(&tx_buffer[13], &tx_msg.imu_y, sizeof(float));
        memcpy(&tx_buffer[17], &tx_msg.imu_z, sizeof(float));
        memcpy(&tx_buffer[21], &tx_msg.imu_yaw, sizeof(float));
        
        // Total message size: 25 bytes
        sendData(TX_TASK_TAG, tx_buffer, 25);
        
        // Delay to achieve 50Hz frequency
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz = 20ms period
    }
}

static void rx_task(void *arg)
{   
    int receiveCount = 0;
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    
    if (data == NULL) {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for RX buffer");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // Read from UART
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, pdMS_TO_TICKS(20)); // Non-blocking
        
        if (rxBytes > 0) {
            data[rxBytes] = 0; // Null terminate for safe printing
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes (UART)", rxBytes);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            
            // Check if it's ASCII text format (contains spaces or commas)
            bool is_ascii_format = false;
            for (int i = 0; i < rxBytes; i++) {
                if (data[i] == ' ' || data[i] == ',') {
                    is_ascii_format = true;
                    break;
                }
            }
            
            if (is_ascii_format) {
                // ASCII text parsing (format: "machine_state,wheel1_speed,wheel2_speed" or "machine_state wheel1_speed wheel2_speed")
                int state = 0, wheel1 = 0, wheel2 = 0;
                int parsed = 0;
                
                if (strchr((char*)data, ',') != NULL) {
                    // Parse comma-separated format
                    parsed = sscanf((char*)data, "%d,%d,%d", &state, &wheel1, &wheel2);
                } else {
                    // Parse space-separated format
                    parsed = sscanf((char*)data, "%d %d %d", &state, &wheel1, &wheel2);
                }
                
                if (parsed >= 1) {
                    rx_msg.machine_state = (uint8_t)state;
                    
                    if (parsed >= 2) {
                        rx_msg.wheel1_speed = (int16_t)wheel1;
                    }
                    
                    if (parsed >= 3) {
                        rx_msg.wheel2_speed = (int16_t)wheel2;
                    }
                    ESP_LOGI(RX_TASK_TAG, "ASCII Parsed: State=%u, Wheel1=%d, Wheel2=%d", 
                            rx_msg.machine_state, rx_msg.wheel1_speed, rx_msg.wheel2_speed);
                }
            }
            else if (rxBytes >= 5) { // Binary format: Machine state (1 byte) + wheel1_speed (2 bytes) + wheel2_speed (2 bytes)
                // Original binary parsing
                rx_msg.machine_state = data[0];
                
                // Parse wheel1_speed (2 bytes) - big endian format
                rx_msg.wheel1_speed = (data[1] << 8) | data[2];
                
                // Parse wheel2_speed (2 bytes) - big endian format
                rx_msg.wheel2_speed = (data[3] << 8) | data[4];
                
                ESP_LOGI(RX_TASK_TAG, "Binary Parsed: State=%u, Wheel1=%d, Wheel2=%d", 
                        rx_msg.machine_state, rx_msg.wheel1_speed, rx_msg.wheel2_speed);
            }
            
            // Echo back the received data through UART to confirm reception
            sendUartData(RX_TASK_TAG, data, rxBytes);
            
            receiveCount = 0; // Reset the receive count on successful read
            
            // Update rx_msg.machine_state to mark when new data was received
            // This helps external tasks detect new data even if actual state value doesn't change
            rx_msg.machine_state = (rx_msg.machine_state & 0x0F) | ((xTaskGetTickCount() & 0x0F) << 4);
            
            if (task_state == IDLE) {
                task_state = RECEIVING;
                ESP_LOGI(RX_TASK_TAG, "Task state changed to RECEIVING");
            }
        }
        else if(receiveCount < 20) {
            receiveCount++;
            // vTaskDelay(pdMS_TO_TICKS(20)); // Keep 50Hz timing
        }

        if (receiveCount >= 20) {
            if (task_state == RECEIVING) {
                task_state = IDLE;
                ESP_LOGI(RX_TASK_TAG, "No data received for 100ms");
                ESP_LOGI(RX_TASK_TAG, "Task state changed to IDLE");
            }
            // vTaskDelay(pdMS_TO_TICKS(100)); // Wait for 100ms before checking again
            continue;
        }
        
        // For non-idle states, maintain the 50Hz timing
        if (receiveCount < 20) {
            // vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    free(data);
}

void uart_init(void)
{
    task_state = RECEIVING;
    
    // Initialize UART
    uart_init_port();
    
    // Create tasks with slightly more stack space to avoid potential overflows
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, 3, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024 * 4, NULL, 3, NULL);
}
