#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stepper_motors.h"
#include "FastAccelStepper.h"
#include "app_motors.h"
#include "esp_log.h"

#define HOME_SPEED_US    2000
#define HOME_MAX_STEPS   10000

#define TAG "STEPPER_MOTORS"

static FastAccelStepperEngine engine;
static FastAccelStepper* stepper = nullptr;

void stepper_motor_task_init(void) {
    engine.init();
    stepper = engine.stepperConnectToPin(STEP_PIN);
    if (stepper) {
        stepper->setDirectionPin(DIR_PIN);
        stepper->setAutoEnable(true);
        stepper->setSpeedInUs(1000);
        stepper->setAcceleration(1000);
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
    stepper->setSpeedInUs(HOME_SPEED_US);
    int32_t startPos = stepper->getCurrentPosition();
    stepper->moveTo(startPos - HOME_MAX_STEPS);
    // Wait for endstop switch to trigger
    while (gpio_get_level((gpio_num_t)ENDSTOP_PIN)) {
        vTaskDelay(1);
    }
    stepper->forceStop();  // Abruptly stop the stepper without deceleration
    stepper->setCurrentPosition(0);
    ESP_LOGI(TAG, "Stepper homed to position 0");
}

void set_stepper_pos(int32_t pos) {
    if (!stepper) return;
    stepper->moveTo(pos);
    ESP_LOGI(TAG, "Moving stepper to position %d", pos);
}