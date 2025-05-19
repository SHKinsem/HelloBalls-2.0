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
#include "serial.h"

#define TAG "MOTORS"

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
    set_servo_position(66);
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
        set_servo_position(66);
    }
    servo_state = !servo_state;
}

can_channel_t can_channel(0, TWAI_TX_PIN, TWAI_RX_PIN); // Create a CAN channel instance
dm3519_t test_motor(1); // Create an instance of the motor

m3508_t frictionWheel_1(1);
m3508_t frictionWheel_2(2);
m3508_t wheelMotor_1(3);
m3508_t wheelMotor_2(4);
m2006_t loaderMotor(6);

void motorStateHelper(bool state) {
    if (state) {
        frictionWheel_1.enable(); // Enable the motor
        frictionWheel_2.enable(); // Enable the motor
        wheelMotor_1.enable(); // Enable the motor
        wheelMotor_2.enable(); // Enable the motor
    } else {
        frictionWheel_1.disable();
        frictionWheel_2.disable();
        wheelMotor_1.disable();
        wheelMotor_2.disable();
    }
}

void serialWheelControlTask(void *arg) {
    ESP_LOGI(TAG, "Serial wheel control task started");
    // Setup for serial activity detection
    rx_message_t* rx_msg = get_rx_message_ptr();
    task_state_t* task_state = getTaskState(); // Get the task state pointer
    
    while (1) {
        if (*task_state == RECEIVING) { motorStateHelper(true); } // Enable the motor if receiving data
        else { motorStateHelper(false); } // Disable the motor if not receiving data

         // Get the machine state from the received message in advance
         // Avoid the state change during the task execution
        uint8_t machine_state = rx_msg->machine_state;
        wheelMotor_1.setTargetSpeed(rx_msg->wheel1_speed);
        wheelMotor_2.setTargetSpeed(-rx_msg->wheel2_speed);        
        if(machine_state == 0) {
            frictionWheel_1.setTargetSpeed(0);
            frictionWheel_2.setTargetSpeed(0);
        } else if (machine_state == 1) {
            frictionWheel_1.setTargetSpeed(1000);
            frictionWheel_2.setTargetSpeed(-1000);
        } else if (machine_state == 2) {
            frictionWheel_1.setTargetSpeed(-1000);
            frictionWheel_2.setTargetSpeed(1000);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void motor_task_init(){
    can_channel.reg_motor(&test_motor);
    can_channel.reg_motor(&frictionWheel_1);
    can_channel.reg_motor(&frictionWheel_2);
    can_channel.reg_motor(&wheelMotor_1);
    can_channel.reg_motor(&wheelMotor_2);

    frictionWheel_1.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters
    frictionWheel_2.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters
    wheelMotor_1.setPIDParameters(30.0, 0.02, 0.1, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters
    wheelMotor_2.setPIDParameters(30.0, 0.02, 0.1, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters

    can_channel.start(); // Start the CAN channel
    // Create the new serial wheel control task

    xTaskCreatePinnedToCore(serialWheelControlTask, "Serial Wheel Control", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 0);
}
