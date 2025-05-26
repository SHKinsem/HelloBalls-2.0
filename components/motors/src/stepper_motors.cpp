#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stepper_motors.h"
#include "FastAccelStepper.h"
#include "app_motors.h"
#include "esp_log.h"
#include <inttypes.h>

#define HOME_SPEED_HZ    2000
#define HOME_MAX_STEPS   20000

#define DEFAULT_SPEED 10000 // Default speed in Hz
#define DEFAULT_ACCELERATION 8000 // Default acceleration in Hz

#define TAG "STEPPER_MOTORS"

static FastAccelStepperEngine engine;
static FastAccelStepper* stepper = nullptr;

void stepper_motor_task_init(void) {
    engine.init();
    stepper = engine.stepperConnectToPin(STEP_PIN);
    if (stepper) {
        stepper->setDirectionPin(DIR_PIN);
        stepper->setAutoEnable(true);
        stepper->setSpeedInHz(DEFAULT_SPEED);
        stepper->setAcceleration(DEFAULT_ACCELERATION);
        stepper->attachToPulseCounter(7);
        ESP_LOGI(TAG, "Stepper initialized successfully");
    } else {
        ESP_LOGE(TAG, "Stepper initialization failed");
    }
}

void home_stepper_motor(void) {
    if (!stepper) return;
    static bool endstop_init = false;
    if (!endstop_init) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = 1ULL << ENDSTOP_PIN;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&io_conf);
        endstop_init = true;
    }
    ESP_LOGI(TAG, "Homing stepper...");
    stepper->setSpeedInHz(HOME_SPEED_HZ); // Set homing speed
    int32_t startPos = stepper->getCurrentPosition();
    stepper->moveTo(HOME_MAX_STEPS - startPos);
    // Wait for endstop switch to trigger
    while (gpio_get_level((gpio_num_t)ENDSTOP_PIN)) {
        vTaskDelay(1);
    }
    stepper->forceStop();  // Abruptly stop the stepper without deceleration
    stepper->setCurrentPosition(0);
    stepper->setSpeedInHz(DEFAULT_SPEED); // Reset speed to default
    ESP_LOGI(TAG, "Stepper homed to position 0");
}


void set_stepper_pos(int32_t pos) {
    if (!stepper) return;
    stepper->moveTo(-pos);
    ESP_LOGI(TAG, "Moving stepper to position %" PRId32, pos);

    // int16_t pcnt = stepper->readPulseCounter();
    // ESP_LOGI(TAG, "Current position: %" PRId32 ", Pulse counter: %d", stepper->getCurrentPosition(), pcnt);
}