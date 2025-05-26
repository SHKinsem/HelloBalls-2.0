#include "serial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "leds.h"
#include "esp32_s3_szp.h"

/*
    RX message format:
    [machine_state, wheel1_speed, wheel2_speed]
    Frequency: 50 Hz

    TX message format:
    [machine_state, wheel1_distance, wheel2_distance, imu_x, imu_y, imu_z, imu_yaw]
    Frequency: 50 Hz
*/

static const int RX_BUF_SIZE = 128;

static serial_state_t task_state = SERIAL_IDEL; // Initialize task state to SERIAL_IDEL

// Global variables for message data
static rx_message_t rx_msg = {0};
static tx_message_t tx_msg = {0};

// Function to get pointer to RX data for external access
rx_message_t* get_rx_message_ptr(void) {
    return &rx_msg;
}

rx_message_t get_rx_message(void) {
    return rx_msg;
}

serial_state_t* getTaskState(void) {
    return &task_state;
}

// Function to set TX data from external source
void set_tx_message(uint8_t state, int32_t w1_dist, int32_t w2_dist, 
                    float x, float y, float z, float yaw) {
}

// Initialize UART communication
static void uart_init_port(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}

// Send data through UART
int sendUartData(const tx_message_t *tx_msg)
{
    // Ensure we have a valid pointer
    if (tx_msg == NULL) {
        return -1;
    }
    
    // Serialize tx_msg into a byte array
    uint8_t data[22]; // Buffer size
    int len = 0;
    const int max_len = sizeof(data);
    
    // Ensure we have enough space for basic fields (safety check)
    if (max_len < 22) { // Minimum size needed for all fields
        return -1;
    }
    
    // Basic fields serialization
    data[len++] = tx_msg->mcu_state;
    data[len++] = tx_msg->host_state;
    
    // Wheel distances (32-bit values)
    data[len++] = (tx_msg->wheel1_distance >> 24) & 0xFF;
    data[len++] = (tx_msg->wheel1_distance >> 16) & 0xFF;
    data[len++] = (tx_msg->wheel1_distance >> 8) & 0xFF;
    data[len++] = tx_msg->wheel1_distance & 0xFF;
    
    data[len++] = (tx_msg->wheel2_distance >> 24) & 0xFF;
    data[len++] = (tx_msg->wheel2_distance >> 16) & 0xFF;
    data[len++] = (tx_msg->wheel2_distance >> 8) & 0xFF;
    data[len++] = tx_msg->wheel2_distance & 0xFF;

    // IMU data serialization (assuming 16-bit values)
    data[len++] = (uint8_t)(tx_msg->imu_data.acc_x >> 8);
    data[len++] = (uint8_t)(tx_msg->imu_data.acc_x & 0xFF);
    
    data[len++] = (uint8_t)(tx_msg->imu_data.acc_y >> 8);
    data[len++] = (uint8_t)(tx_msg->imu_data.acc_y & 0xFF);
    
    data[len++] = (uint8_t)(tx_msg->imu_data.acc_z >> 8);
    data[len++] = (uint8_t)(tx_msg->imu_data.acc_z & 0xFF);
    
    data[len++] = (uint8_t)(tx_msg->imu_data.gyr_x >> 8);
    data[len++] = (uint8_t)(tx_msg->imu_data.gyr_x & 0xFF);
    
    data[len++] = (uint8_t)(tx_msg->imu_data.gyr_y >> 8);
    data[len++] = (uint8_t)(tx_msg->imu_data.gyr_y & 0xFF);
    
    data[len++] = (uint8_t)(tx_msg->imu_data.gyr_z >> 8);
    data[len++] = (uint8_t)(tx_msg->imu_data.gyr_z & 0xFF);

    // Send the data through UART and handle errors
    int bytes_sent = uart_write_bytes(UART_NUM_0, (const char *)data, len);
    if (bytes_sent < 0 || bytes_sent != len) {
        ESP_LOGE("UART_SEND", "Failed to send data: expected %d bytes, sent %d bytes", len, bytes_sent);
        // Error handling
        return -1;
    }
    
    return bytes_sent;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    
    tx_msg.mcu_state = 0; // Default state
    
    while (1) {
        // if (task_state == SERIAL_IDEL) {
        //     vTaskDelay(pdMS_TO_TICKS(500));
        //     continue;
        // }

        qmi8658_Read_AccAndGry(&tx_msg.imu_data); // Fetch IMU data

        sendUartData(&tx_msg); // Send the tx_msg data through UART
        // ESP_LOGI(TX_TASK_TAG, "Sent: State=%u, Wheel1Dist=%"PRIu32", Wheel2Dist=%"PRIu32", IMU Acc=(%d, %d, %d), Gyr=(%d, %d, %d)", 
        //         tx_msg.mcu_state, tx_msg.wheel1_distance, tx_msg.wheel2_distance,
        //         tx_msg.imu_data.acc_x, tx_msg.imu_data.acc_y, tx_msg.imu_data.acc_z,
        //         tx_msg.imu_data.gyr_x, tx_msg.imu_data.gyr_y, tx_msg.imu_data.gyr_z);
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

            bool is_ascii_format = false;
            for (int i = 0; i < rxBytes; i++) {
                if (data[i] == ' ' || data[i] == ',') {
                    is_ascii_format = true;
                    break;
                }
            }
            
            if (is_ascii_format) {
                // ASCII text parsing (format: "machine_state,wheel1_speed,wheel2_speed" or "machine_state wheel1_speed wheel2_speed")
                int state = 0, wheel1 = 0, wheel2 = 0, servoAngle = 0;
                int parsed = 0;
                
                if (strchr((char*)data, ',') != NULL) 
                    parsed = sscanf((char*)data, "%d,%d,%d,%d", &state, &wheel1, &wheel2, &servoAngle);
                
                if (parsed >= 1) {
                    rx_msg.host_state = (uint8_t)state;
                    if (parsed >= 2) rx_msg.wheel1_speed = (int16_t)wheel1;
                    if (parsed >= 3) rx_msg.wheel2_speed = (int16_t)wheel2;
                    if (parsed >= 4) rx_msg.tilt_angle = (int16_t)servoAngle;
                    ESP_LOGI(RX_TASK_TAG, "ASCII Parsed: State=%u, Wheel1=%d, Wheel2=%d, Tilt=%d", 
                            rx_msg.host_state, rx_msg.wheel1_speed, rx_msg.wheel2_speed, rx_msg.tilt_angle);
                }
            }
            else if (rxBytes >= 7) {
                rx_msg.host_state =     data[0];
                rx_msg.wheel1_speed =   (data[1] << 8) | data[2];
                rx_msg.wheel2_speed =   (data[3] << 8) | data[4];
                rx_msg.tilt_angle =     (data[5] << 8) | data[6];

                ESP_LOGI(RX_TASK_TAG, "Binary Parsed: State=%u, Wheel1=%d, Wheel2=%d, Tilt=%d", 
                        rx_msg.host_state, rx_msg.wheel1_speed, rx_msg.wheel2_speed, rx_msg.tilt_angle);
            }
            // Echo back the received data through UART to confirm reception
            // sendUartData(RX_TASK_TAG, data, rxBytes);
            receiveCount = 0; // Reset the receive count on successful read

            if (task_state == SERIAL_IDEL) {
                task_state = SERIAL_RECEIVING;
                ESP_LOGI(RX_TASK_TAG, "Task state changed to SERIAL_RECEIVING");
            }
            update_led_state_noHandle(SEARCHING_BALL); // Update LED state to SERIAL_RECEIVING
        }
        else if(receiveCount < 20) {
            receiveCount++;
            // vTaskDelay(pdMS_TO_TICKS(20)); // Keep 50Hz timing
        }

        if (receiveCount >= 20) {
            if (task_state == SERIAL_RECEIVING) {
                task_state = SERIAL_IDEL;
                ESP_LOGI(RX_TASK_TAG, "No data received for 100ms");
                ESP_LOGI(RX_TASK_TAG, "Task state changed to SERIAL_IDEL");
            }
            // vTaskDelay(pdMS_TO_TICKS(100)); // Wait for 100ms before checking again
            update_led_state_noHandle(ERROR); // Update LED state to SERIAL_IDEL
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
    task_state = SERIAL_RECEIVING;
    
    // Initialize UART
    uart_init_port();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, 3, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024 * 4, NULL, 3, NULL);
}
