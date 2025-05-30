#pragma once

#ifndef SERVO_H
#define SERVO_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp32_s3_szp.h"
#include <stdint.h>



// Servo control parameters - use different GPIO and timer/channel combination
#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define CHANNEL_SERVO_1         LEDC_CHANNEL_1
#define CHANNEL_SERVO_2         LEDC_CHANNEL_2
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          50                // PWM frequency in Hz (standard for servo is 50Hz)
#define SERVO_MIN_PULSEWIDTH    500               // Minimum pulse width in microseconds (0 degrees)
#define SERVO_MAX_PULSEWIDTH    2500              // Maximum pulse width in microseconds (180 degrees)
#define SERVO_ANGLE_0           0                 // 0 degree position
#define SERVO_ANGLE_30          30                // 30 degree position
#define TAG "SERVO"

class ServoController {

private:
    gpio_num_t pin;
    ledc_channel_t channel;
    ledc_timer_t timer;
    ledc_mode_t mode = LEDC_LOW_SPEED_MODE; // Use low speed mode for servo control

    uint32_t current_duty = 0; // Current duty cycle for the servo
    uint32_t origin_duty = 0; // Origin duty cycle for the servo
    bool enabled = true;      // Track if the servo is enabled
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

    void release() {
        if(!enabled) return; // Can't release if already disabled
        
        // Set duty to zero to stop the servo from actively holding position
        ESP_ERROR_CHECK(ledc_set_duty(mode, channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(mode, channel));
        current_duty = 0; // Update current duty
        ESP_LOGI(TAG, "Servo on GPIO%d released (not actively holding position)", pin);
    }
    
    void disable(bool release_first = true) {
        if(!enabled) return; // Already disabled
        
        if(release_first) {
            release(); // First release the servo to stop it from holding position
        }
        
        ESP_ERROR_CHECK(ledc_stop(mode, channel, 0)); // Stop PWM output
        enabled = false;
        ESP_LOGI(TAG, "Servo on GPIO%d disabled", pin);
    }
    
    void enable() {
        if(enabled) return; // Already enabled
        enabled = true;
        ESP_LOGI(TAG, "Servo on GPIO%d enabled, restored to duty: %"PRIu32, pin, current_duty);
    }
    
    bool isEnabled() const {
        return enabled;
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

    void move(uint32_t duty, bool verbose = true) {
        if(!enabled) {
            ESP_LOGW(TAG, "Servo on GPIO%d is disabled. Cannot move.", pin);
            return;
        }
        
        if(current_duty == duty) {
            return; // No need to move if already at the desired duty cycle
        }
        // Move servo to specified duty cycle
        ESP_ERROR_CHECK(ledc_set_duty(mode, channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(mode, channel));
        current_duty = duty; // Update current duty
        if (verbose) {
            float angle = duty2angle(duty); // Convert duty cycle to angle for logging
            ESP_LOGI(TAG, "Servo on GPIO%d moved to angle: %.2f degrees, duty: %"PRIu32, pin, angle, duty);
        }
    }

    void reset() {
        move(origin_duty); // Move servo to origin duty cycle
        ESP_LOGI(TAG, "Servo on GPIO %d reset to origin duty: %"PRIu32, pin, origin_duty);
    }

    void setOriginDuty(uint32_t duty) {
        origin_duty = duty;
    }

    void setOriginAngle(float angle) {
        // Convert angle to duty cycle and set as origin
        uint32_t duty = angle2duty(angle);
        setOriginDuty(duty);
        ESP_LOGI(TAG, "Servo on GPIO %d origin angle set to: %.2f degrees, duty: %"PRIu32, pin, angle, duty);
    }

    uint32_t getCurrentDuty() const {
        return current_duty;
    }

    uint32_t getOriginDuty() const {
        return origin_duty;
    }

    void setAngle(float angle) {
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

    void moveRelativeToOrigin(float angle, float speed) {
        // Validate speed parameter
        if (speed <= 0) {
            ESP_LOGW(TAG, "Invalid speed value: %.2f, using default speed of 1.0", speed);
            speed = 1.0f;
        }
        
        // Calculate new angle relative to origin
        float origin_angle = duty2angle(getOriginDuty());
        float new_angle = origin_angle + angle;
        
        // Clamp angle to valid range
        if (new_angle < 0) new_angle = 0;
        if (new_angle > 180) new_angle = 180;
        
        // Calculate target duty cycle based on clamped angle
        uint32_t target_duty = angle2duty(new_angle);
        uint32_t current_duty = getCurrentDuty();
        
        // Early exit if already at target position
        if (current_duty == target_duty) {
            ESP_LOGI(TAG, "Servo on GPIO%d already at target position", pin);
            return;
        }
        
        ESP_LOGI(TAG, "Moving servo on GPIO%d relative to origin by %.2f degrees, new angle: %.2f with speed: %.2f", 
                 pin, angle, new_angle, speed);
        
        // Move servo gradually to target position
        while (current_duty != target_duty) {
            if (current_duty < target_duty) current_duty++;
            else current_duty--;
            
            move(current_duty, false); // Move servo to current duty cycle
            vTaskDelay(pdMS_TO_TICKS(100 / speed));
        }
    }
};

#endif // SERVO_H