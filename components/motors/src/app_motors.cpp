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
#include <cmath>
// #include "ui.h"
#include "app_twai.h"
#include "app_motors.h"
#include "esp_task_wdt.h"
#include "serial.h"
#include <algorithm>  // For std::min and std::max

#define TAG "MOTORS"

#define SERVO_1_MIDDLE_ANGLE 90.0f // Middle position for servo 1
#define SERVO_2_MIDDLE_ANGLE 91.0f // Middle position for servo 2

class ServoController {

private:
    gpio_num_t pin;
    ledc_channel_t channel;
    ledc_timer_t timer;
    ledc_mode_t mode = LEDC_LOW_SPEED_MODE; // Use low speed mode for servo control

    uint32_t current_duty = 0; // Current duty cycle for the servo
    uint32_t origin_duty = 0; // Origin duty cycle for the servo
public:
    ServoController(gpio_num_t pin, ledc_channel_t channel, ledc_timer_t timer) : pin(pin), channel(channel), timer(timer) {
        // Initialize the LEDC timer
        ledc_timer_config_t ledc_timer = {
            .speed_mode = mode,
            .duty_resolution = LEDC_TIMER_13_BIT, // 13-bit resolution
            .timer_num = timer,
            .freq_hz = 50, // Standard servo frequency
            .clk_cfg = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
        
        // Initialize the LEDC channel for servo control
        ledc_channel_config_t ledc_channel = {
            .gpio_num = pin,
            .speed_mode = mode,
            .channel = channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer,
            .duty = 0, // Initial duty cycle set to 0
            .hpoint = 0,
            .flags = {}
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
    ~ServoController() {
        // Cleanup if needed
        ESP_ERROR_CHECK(ledc_stop(mode, channel, 0));
    }

    uint32_t angle2duty(float angle) {
        // Convert angle to a value between 0-180
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        
        // Calculate pulse width based on angle
        float pulse_width = SERVO_MIN_PULSEWIDTH + (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (angle / 180.0);
        
        // Convert pulse width to duty cycle
        uint32_t duty = (uint32_t)((pulse_width / 20000.0) * ((1 << LEDC_DUTY_RES) - 1));
        return duty;
    }
    float duty2angle(uint32_t duty) {
        // Convert duty cycle back to angle
        // First convert duty to pulse width
        float pulse_width = (duty * 20000.0) / ((1 << LEDC_DUTY_RES) - 1);
        
        if (pulse_width < SERVO_MIN_PULSEWIDTH || pulse_width > SERVO_MAX_PULSEWIDTH) {
            ESP_LOGW(TAG, "Pulse width out of range: %.2f", pulse_width);
            return -1.0f; // Invalid pulse width
        }
        
        // Calculate angle based on pulse width
        float angle = (pulse_width - SERVO_MIN_PULSEWIDTH) / (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * 180.0;
        return std::max(0.0f, std::min(angle, 180.0f)); // Clamp angle between 0 and 180
    }

    void move(uint32_t duty) {
        if(current_duty == duty) {
            return; // No need to move if already at the desired duty cycle
        }
        // Move servo to specified duty cycle
        ESP_ERROR_CHECK(ledc_set_duty(mode, channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(mode, channel));
        current_duty = duty; // Update current duty
        ESP_LOGI(TAG, "Servo on GPIO%d moved to duty: %"PRIu32, pin, duty);
    }

    void reset() {
        move(origin_duty); // Move servo to origin duty cycle
        ESP_LOGI(TAG, "Servo on GPIO%d reset to origin duty: %"PRIu32, pin, origin_duty);
    }

    void setOriginDuty(uint32_t duty) {
        origin_duty = duty;
    }

    void setOriginAngle(float angle) {
        // Convert angle to duty cycle and set as origin
        uint32_t duty = angle2duty(angle);
        setOriginDuty(duty);
        ESP_LOGI(TAG, "Servo on GPIO%d origin angle set to: %.2f degrees, duty: %"PRIu32, pin, angle, duty);
    }

    uint32_t getCurrentDuty() const {
        return current_duty;
    }

    uint32_t getOriginDuty() const {
        return origin_duty;
    }

    void setAngle(float angle) {
        // Convert angle to duty cycle
        uint32_t duty = angle2duty(angle);
        move(duty); // Move servo to calculated duty cycle
    }   

    void moveRelativeToOrigin(float angle) {
        // Move servo relative to origin position
        float new_angle = duty2angle(getOriginDuty()) + angle; // Get current angle from origin duty
        ESP_LOGI(TAG, "Moving servo on GPIO%d relative to origin by %.2f degrees, new angle: %.2f", pin, angle, new_angle);
        if (new_angle < 0) new_angle = 0;
        if (new_angle > 180) new_angle = 180;
        setAngle(new_angle);
    }
};

ServoController servo1(SERVO_1_PIN, CHANNEL_SERVO_1, LEDC_TIMER);
ServoController servo2(SERVO_2_PIN, CHANNEL_SERVO_2, LEDC_TIMER);

// Initialize servo on GPIO18
void servo_init(void) {
    ESP_LOGI(TAG, "Initializing servo on GPIO%d", SERVO_1_PIN);
    servo1.setOriginAngle(SERVO_1_MIDDLE_ANGLE); // Set origin angle for servo 1
    servo2.setOriginAngle(SERVO_2_MIDDLE_ANGLE); // Set origin angle for servo 2

    servo1.reset(); // Reset servo 1 to origin position
    servo2.reset(); // Reset servo 2 to origin position
    
    ESP_LOGI(TAG, "Servo initialized to 90 degrees position");
}


void set_servo_position(float angle) {
    servo1.setAngle(angle);
    servo2.setAngle(180 - angle);
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay to allow servo to move
}

void set_servos_position(float angle1, float angle2) {
    servo1.setAngle(angle1);
    servo2.setAngle(angle2);
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay to allow servos to move
}

bool servo_state = true;
// Toggle servo position - helper function for C code
void toggle_servo(void) {
    if(servo_state) {
        servo1.moveRelativeToOrigin(-30.0f); // Move servo 1 to 30 degrees relative to origin
        servo2.moveRelativeToOrigin(30.0f); // Move servo 2 to -30 degrees relative to origin
    } else {
        servo1.moveRelativeToOrigin(3.0f); 
        servo2.moveRelativeToOrigin(-3.0f); 
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
