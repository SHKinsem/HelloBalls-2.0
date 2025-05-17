#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/ledc.h"
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

// Servo control parameters - use different GPIO and timer/channel combination
#define SERVO_GPIO              SERVO_PIN         // Changed from GPIO0 to GPIO18
#define LEDC_TIMER              LEDC_TIMER_1  // Changed from TIMER_0 to TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_1  // Changed from CHANNEL_0 to CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          50                // PWM frequency in Hz (standard for servo is 50Hz)
#define SERVO_MIN_PULSEWIDTH    500               // Minimum pulse width in microseconds (0 degrees)
#define SERVO_MAX_PULSEWIDTH    2500              // Maximum pulse width in microseconds (180 degrees)
#define SERVO_ANGLE_0           0                 // 0 degree position
#define SERVO_ANGLE_30          30                // 30 degree position

// Convert angle to duty cycle for servo control
static uint32_t servo_angle_to_duty(float angle) {
    // Convert angle to a value between 0-180
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Calculate pulse width based on angle
    float pulse_width = SERVO_MIN_PULSEWIDTH + (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (angle / 180.0);
    
    // Convert pulse width to duty cycle
    uint32_t duty = (uint32_t)((pulse_width / 20000.0) * ((1 << LEDC_DUTY_RES) - 1));
    return duty;
}

// Initialize servo on GPIO18
void servo_init(void) {
    ESP_LOGI(TAG, "Initializing servo on GPIO%d", SERVO_GPIO);
    
    // LEDC timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // Channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = SERVO_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Initial duty cycle set to 0
        .hpoint         = 0,
        .flags          = {}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    // Set initial position to 0 degrees
    set_servo_position(60);
    ESP_LOGI(TAG, "Servo initialized to 0 degrees position");
}


void set_servo_position(float angle) {
    uint32_t duty = servo_angle_to_duty(angle);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

bool servo_state = true;
// Toggle servo position - helper function for C code
void toggle_servo(void) {
    if(servo_state) {
        set_servo_position(20);
    } else {
        set_servo_position(60);
    }
    servo_state = !servo_state;
}

m3508_t frictionwheels[2] = {m3508_t(1), m3508_t(2)}; // Create instances of m3508 motors for friction wheels
m3508_t wheels[2] = {m3508_t(3), m3508_t(4)}; // Create instances of m3508 motors for wheels
// m2006_t loaderMotor(3); // Create an instance of m2006 motor for loader motor

base_motor_t* get_motor_ptr(uint8_t motor_id) {
    switch (motor_id) {
        case 1:
            return &frictionwheels[0];
        case 2:
            return &frictionwheels[1];
        case 3:
            return &wheels[0];
        case 4:
            return &wheels[1];
        default:
            ESP_LOGE(TAG, "Invalid motor ID: %d", motor_id);
            return NULL; // Invalid motor ID
    }
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
            // loaderMotor.parseData(rx_msg.data); // Parse data for the loader motor
            wheels[0].parseData(rx_msg.data); // Example for parsing wheel data
        } else if(msg_id == 4) {
            // Handle DM3519 motor data here if needed
            // dm3519_motor.parseData(rx_msg.data); // Example for parsing DM3519 motor data
            wheels[1].parseData(rx_msg.data); // Example for parsing wheel data
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


