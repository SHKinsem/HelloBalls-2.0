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
#include "cmd_hub.h"

/*
    RX message format (ASCII, one command per line):
    "state,wheel1_speed,wheel2_speed[,tilt_angle,shoot_speed]\n"
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

#define RX_LINE_MAX 64

static serial_state_t task_state = SERIAL_IDEL; // Initialize task state to SERIAL_IDEL
static mcu_state_t mcu_state = MCU_IDLE; // Initialize MCU state to MCU_IDLE

// Global variables for message data
static robot_cmd_t rx_cmd = {0};
static tx_message_t tx_msg = {0};
static portMUX_TYPE tx_msg_lock = portMUX_INITIALIZER_UNLOCKED;

serial_state_t* getTaskState(void) {
    return &task_state;
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

void get_tx_wheel_speed_feedback(int32_t *wheel1_speed_rpm,
                                 int32_t *wheel2_speed_rpm) {
    taskENTER_CRITICAL(&tx_msg_lock);
    if (wheel1_speed_rpm) *wheel1_speed_rpm = tx_msg.wheel1_speed_rpm;
    if (wheel2_speed_rpm) *wheel2_speed_rpm = tx_msg.wheel2_speed_rpm;
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
        sample.mcu_state = (uint8_t)mcu_state;
        sample.host_state = cmd_hub_host_state();
        qmi8658_Read_AccAndGry(&sample.imu_data);

        sendUartData(&sample);
        // ESP_LOGI(TX_TASK_TAG, "Sent: State=%u, Wheel1Speed=%"PRId32", Wheel2Speed=%"PRId32", IMU Acc=(%d, %d, %d), Gyr=(%d, %d, %d)",
        //         sample.mcu_state, sample.wheel1_speed_rpm, sample.wheel2_speed_rpm,
        //         tx_msg.imu_data.acc_x, tx_msg.imu_data.acc_y, tx_msg.imu_data.acc_z,
        //         tx_msg.imu_data.gyr_x, tx_msg.imu_data.gyr_y, tx_msg.imu_data.gyr_z);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
    }
}

// Parse one complete ASCII command line and submit it to cmd_hub
static void handle_line(const char *line)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    int state = 0, wheel1 = 0, wheel2 = 0, servoAngle = 0, shootSpeed = 0;
    int parsed = sscanf(line, "%d,%d,%d,%d,%d", &state, &wheel1, &wheel2, &servoAngle, &shootSpeed);
    if (parsed < 1) {
        ESP_LOGW(RX_TASK_TAG, "Ignoring malformed command: %s", line);
        return;
    }

    // Fields that were not sent keep their previous values (3-field commands)
    rx_cmd.host_state = (uint8_t)state;
    if (parsed >= 2) rx_cmd.wheel1_speed = (int16_t)wheel1;
    if (parsed >= 3) rx_cmd.wheel2_speed = (int16_t)wheel2;
    if (parsed >= 4) rx_cmd.tilt_angle = (int16_t)servoAngle;
    if (parsed >= 5) rx_cmd.shoot_speed = (int16_t)shootSpeed;
    ESP_LOGD(RX_TASK_TAG, "Parsed: State=%u, Wheel1=%d, Wheel2=%d, Tilt=%d, Shoot=%d",
             rx_cmd.host_state, rx_cmd.wheel1_speed, rx_cmd.wheel2_speed, rx_cmd.tilt_angle, rx_cmd.shoot_speed);
    cmd_hub_submit(CMD_SRC_SERIAL, &rx_cmd);
}

static void rx_task(void *arg)
{
    int receiveCount = 0;
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE);
    char line[RX_LINE_MAX];
    int line_len = 0;
    bool line_overflow = false;

    if (data == NULL) {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for RX buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, pdMS_TO_TICKS(20));

        if (rxBytes > 0) {
            /*
             * Commands may arrive split across reads or several per read, so
             * only complete lines are parsed (fixes half commands read as state=0).
             */
            for (int i = 0; i < rxBytes; i++) {
                char c = (char)data[i];
                if (c == '\n' || c == '\r') {
                    if (line_len > 0 && !line_overflow) {
                        line[line_len] = '\0';
                        handle_line(line);
                    }
                    line_len = 0;
                    line_overflow = false;
                } else if (line_len < RX_LINE_MAX - 1) {
                    line[line_len++] = c;
                } else {
                    line_overflow = true; // drop the rest of an over-long line
                }
            }

            receiveCount = 0; // Reset the receive count on successful read
            if (task_state == SERIAL_IDEL) {
                task_state = SERIAL_RECEIVING;
                ESP_LOGI(RX_TASK_TAG, "Task state changed to SERIAL_RECEIVING");
            }
            update_led_state_noHandle(SEARCHING_BALL); // Update LED state to SERIAL_RECEIVING
        }
        else if(receiveCount < 20) {
            receiveCount++;
        }

        if (receiveCount >= 20) {
            if (task_state == SERIAL_RECEIVING) {
                task_state = SERIAL_IDEL;
                ESP_LOGI(RX_TASK_TAG, "No data received for 400ms");
                ESP_LOGI(RX_TASK_TAG, "Task state changed to SERIAL_IDEL");
            }
            cmd_hub_status_t hub;
            cmd_hub_get_status(&hub);
            if (!(hub.takeover && hub.fresh[CMD_SRC_BLE])) {
                update_led_state_noHandle(ERROR); // No host and no BLE remote driving
            }
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
