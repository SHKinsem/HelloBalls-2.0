#include "serial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
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
    [0xAA, 0x55, msg_type, payload_len, payload..., checksum]
    Frequency: 100 Hz
*/

static const int RX_BUF_SIZE = 128;

#define UART_TX_FRAME_HEADER_0  0xAA
#define UART_TX_FRAME_HEADER_1  0x55
#define UART_TX_MSG_TYPE_IMU_V2 0x02
#define UART_TX_PAYLOAD_LEN_V2  34
#define IMU_SAMPLE_PERIOD_MS    10
#define UART_TX_FRAME_LEN_V2    (2 + 1 + 1 + UART_TX_PAYLOAD_LEN_V2 + 1)

SemaphoreHandle_t serial_rx_semaphore = NULL; // Semaphore for serial RX task synchronization
SemaphoreHandle_t state_change_semaphore = NULL; // Semaphore for state change synchronization

static serial_state_t task_state = SERIAL_IDEL; // Initialize task state to SERIAL_IDEL
static host_state_t host_state = HOST_IDLE; // Initialize host state to HOST_IDLE
static mcu_state_t mcu_state = MCU_IDLE; // Initialize MCU state to MCU_IDLE

// Global variables for message data
static rx_message_t rx_msg = {0};
static tx_message_t tx_msg = {0};
static portMUX_TYPE tx_msg_lock = portMUX_INITIALIZER_UNLOCKED;

// Function to get pointer to RX data for external access
rx_message_t* getRXmsg(void) {
    return &rx_msg;
}

rx_message_t get_rx_message(void) {
    return rx_msg;
}

serial_state_t* getTaskState(void) {
    return &task_state;
}

host_state_t* getHostState(void) {
    return &host_state;
}

mcu_state_t* getMcuState(void) {
    return &mcu_state;
}

void set_tx_wheel_speed_feedback(int32_t wheel1_speed_rpm,
                                 int32_t wheel2_speed_rpm) {
    taskENTER_CRITICAL(&tx_msg_lock);
    tx_msg.wheel1_speed_rpm = wheel1_speed_rpm;
    tx_msg.wheel2_speed_rpm = wheel2_speed_rpm;
    taskEXIT_CRITICAL(&tx_msg_lock);
}

static void append_u8(uint8_t *data, int *len, uint8_t value)
{
    data[(*len)++] = value;
}

static void append_i16_be(uint8_t *data, int *len, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    data[(*len)++] = (uint8_t)((raw >> 8) & 0xFF);
    data[(*len)++] = (uint8_t)(raw & 0xFF);
}

static void append_i32_be(uint8_t *data, int *len, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[(*len)++] = (uint8_t)((raw >> 24) & 0xFF);
    data[(*len)++] = (uint8_t)((raw >> 16) & 0xFF);
    data[(*len)++] = (uint8_t)((raw >> 8) & 0xFF);
    data[(*len)++] = (uint8_t)(raw & 0xFF);
}

static void append_u32_be(uint8_t *data, int *len, uint32_t value)
{
    data[(*len)++] = (uint8_t)((value >> 24) & 0xFF);
    data[(*len)++] = (uint8_t)((value >> 16) & 0xFF);
    data[(*len)++] = (uint8_t)((value >> 8) & 0xFF);
    data[(*len)++] = (uint8_t)(value & 0xFF);
}

static void append_u64_be(uint8_t *data, int *len, uint64_t value)
{
    data[(*len)++] = (uint8_t)((value >> 56) & 0xFF);
    data[(*len)++] = (uint8_t)((value >> 48) & 0xFF);
    data[(*len)++] = (uint8_t)((value >> 40) & 0xFF);
    data[(*len)++] = (uint8_t)((value >> 32) & 0xFF);
    data[(*len)++] = (uint8_t)((value >> 24) & 0xFF);
    data[(*len)++] = (uint8_t)((value >> 16) & 0xFF);
    data[(*len)++] = (uint8_t)((value >> 8) & 0xFF);
    data[(*len)++] = (uint8_t)(value & 0xFF);
}

static uint8_t calc_checksum(const uint8_t *data, int start, int end)
{
    uint8_t checksum = 0;
    for (int i = start; i < end; i++) {
        checksum = (uint8_t)(checksum + data[i]);
    }
    return checksum;
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
    
    uint8_t data[UART_TX_FRAME_LEN_V2];
    int len = 0;

    append_u8(data, &len, UART_TX_FRAME_HEADER_0);
    append_u8(data, &len, UART_TX_FRAME_HEADER_1);
    append_u8(data, &len, UART_TX_MSG_TYPE_IMU_V2);
    append_u8(data, &len, UART_TX_PAYLOAD_LEN_V2);

    append_u8(data, &len, tx_msg->mcu_state);
    append_u8(data, &len, tx_msg->host_state);
    append_i32_be(data, &len, tx_msg->wheel1_speed_rpm);
    append_i32_be(data, &len, tx_msg->wheel2_speed_rpm);
    append_i16_be(data, &len, tx_msg->imu_data.acc_x);
    append_i16_be(data, &len, tx_msg->imu_data.acc_y);
    append_i16_be(data, &len, tx_msg->imu_data.acc_z);
    append_i16_be(data, &len, tx_msg->imu_data.gyr_x);
    append_i16_be(data, &len, tx_msg->imu_data.gyr_y);
    append_i16_be(data, &len, tx_msg->imu_data.gyr_z);
    append_u32_be(data, &len, tx_msg->sample_sequence);
    append_u64_be(data, &len, tx_msg->sample_time_us);
    append_u8(data, &len, calc_checksum(data, 2, len));

    if (len != UART_TX_FRAME_LEN_V2) {
        ESP_LOGE("UART_SEND", "Invalid v2 frame length: expected %d bytes, got %d bytes",
                 UART_TX_FRAME_LEN_V2, len);
        return -1;
    }

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
    
    uint32_t sample_sequence = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // if (task_state == SERIAL_IDEL) {
        //     vTaskDelay(pdMS_TO_TICKS(500));
        //     continue;
        // }

        /*
         * Capture the sample time at the sensor-read point, not when UART
         * transmission starts.  Keep all per-sample fields in this local
         * snapshot so a later sample cannot modify a frame being serialized.
         */
        tx_message_t sample;
        taskENTER_CRITICAL(&tx_msg_lock);
        sample = tx_msg;
        taskEXIT_CRITICAL(&tx_msg_lock);
        sample.sample_time_us = (uint64_t)esp_timer_get_time();
        sample.sample_sequence = sample_sequence++;
        qmi8658_Read_AccAndGry(&sample.imu_data);

        sendUartData(&sample);
        // ESP_LOGI(TX_TASK_TAG, "Sent: State=%u, Wheel1Speed=%"PRId32", Wheel2Speed=%"PRId32", IMU Acc=(%d, %d, %d), Gyr=(%d, %d, %d)",
        //         sample.mcu_state, sample.wheel1_speed_rpm, sample.wheel2_speed_rpm,
        //         tx_msg.imu_data.acc_x, tx_msg.imu_data.acc_y, tx_msg.imu_data.acc_z,
        //         tx_msg.imu_data.gyr_x, tx_msg.imu_data.gyr_y, tx_msg.imu_data.gyr_z);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
    }
}

static void rx_task(void *arg)
{   
    if (serial_rx_semaphore == NULL) {
        ESP_LOGE("RX_TASK", "Failed to create serial_rx_semaphore");
        vTaskDelete(NULL);
        return;
    }
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
                int state = 0, wheel1 = 0, wheel2 = 0, servoAngle = 0, shootSpeed = 0;
                int parsed = 0;

                if (strchr((char*)data, ',') != NULL)
                    parsed = sscanf((char*)data, "%d,%d,%d,%d,%d", &state, &wheel1, &wheel2, &servoAngle, &shootSpeed);

                if (parsed >= 1) {
                    if(host_state != state) {
                        host_state = (host_state_t)state; // Update host state
                        xSemaphoreGive(state_change_semaphore); // Signal state change
                    }
                    rx_msg.host_state = (uint8_t)state;
                    if (parsed >= 2) rx_msg.wheel1_speed = (int16_t)wheel1;
                    if (parsed >= 3) rx_msg.wheel2_speed = (int16_t)wheel2;
                    if (parsed >= 4) rx_msg.tilt_angle = (int16_t)servoAngle;
                    if (parsed >= 5) rx_msg.shoot_speed = (int16_t)shootSpeed;
                    ESP_LOGI(RX_TASK_TAG, "ASCII Parsed: State=%u, Wheel1=%d, Wheel2=%d, Tilt=%d, Shoot=%d", 
                            rx_msg.host_state, rx_msg.wheel1_speed, rx_msg.wheel2_speed, rx_msg.tilt_angle, rx_msg.shoot_speed);
                }
            } else if (rxBytes >= 7) {
                rx_msg.host_state =     data[0];
                rx_msg.wheel1_speed =   (data[1] << 8) | data[2];
                rx_msg.wheel2_speed =   (data[3] << 8) | data[4];
                rx_msg.tilt_angle =     (data[5] << 8) | data[6];
                rx_msg.shoot_speed =    (data[7] << 8) | data[8];

                ESP_LOGI(RX_TASK_TAG, "Binary Parsed: State=%u, Wheel1=%d, Wheel2=%d, Tilt=%d, Shoot=%d", 
                        rx_msg.host_state, rx_msg.wheel1_speed, rx_msg.wheel2_speed, rx_msg.tilt_angle, rx_msg.shoot_speed);
            } else {
                ESP_LOGW(RX_TASK_TAG, "Received data in unexpected format: %.*s", rxBytes, data);
            }
            
            receiveCount = 0; // Reset the receive count on successful read
            if (task_state == SERIAL_IDEL) {
                task_state = SERIAL_RECEIVING;
                ESP_LOGI(RX_TASK_TAG, "Task state changed to SERIAL_RECEIVING");
            }
            xSemaphoreGive(serial_rx_semaphore); // Signal that data has been received
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
    vTaskDelete(NULL); // Delete the task if it exits
}

void uart_init(void)
{
    task_state = SERIAL_RECEIVING;
    
    // Initialize UART
    uart_init_port();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, 3, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 4, NULL, 3, NULL);
}
