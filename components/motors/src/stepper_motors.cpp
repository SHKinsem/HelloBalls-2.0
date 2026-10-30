#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stepper_motors.h"
#include "FastAccelStepper.h"
#include "app_motors.h"
#include "esp_log.h"
#include <inttypes.h>

#define HOME_SPEED_HZ    3000
#define HOME_MAX_STEPS   5000
#define HOME_TIMEOUT_MS  10000 // Give up if the endstop never triggers (unplugged switch, no stepper power)

#define DEFAULT_SPEED 10000 // Default speed in Hz
#define DEFAULT_ACCELERATION 10000 // Default acceleration in Hz

#define TAG "STEPPER_MOTORS"

static FastAccelStepperEngine engine;
static FastAccelStepper* stepper = nullptr;
static volatile bool homed = false;

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

bool home_stepper_motor(void) {
    if (!stepper) return false;
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
    // Wait for endstop switch to trigger, but never forever
    TickType_t start = xTaskGetTickCount();
    bool hit = true;
    while (gpio_get_level((gpio_num_t)ENDSTOP_PIN)) {
        if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(HOME_TIMEOUT_MS)) {
            hit = false;
            break;
        }
        vTaskDelay(1);
    }
    stepper->forceStop();  // Abruptly stop the stepper without deceleration
    stepper->setSpeedInHz(DEFAULT_SPEED); // Reset speed to default
    if (!hit) {
        homed = false;
        ESP_LOGE(TAG, "Homing timed out after %d ms: endstop on GPIO%d never triggered", HOME_TIMEOUT_MS, ENDSTOP_PIN);
        return false;
    }
    stepper->setCurrentPosition(0);
    homed = true;
    ESP_LOGI(TAG, "Stepper homed to position 0");
    return true;
}

bool stepper_is_homed(void) {
    return homed;
}


void set_stepper_pos(int32_t pos) {
    if (!stepper) return;
    if (!homed) {
        // Position is unknown, moving could drive the carriage into the frame
        ESP_LOGW(TAG, "Stepper not homed, ignoring move to %" PRId32, pos);
        return;
    }
    stepper->moveTo(-pos);
    ESP_LOGI(TAG, "Moving stepper to position %" PRId32, pos);

    // int16_t pcnt = stepper->readPulseCounter();
    // ESP_LOGI(TAG, "Current position: %" PRId32 ", Pulse counter: %d", stepper->getCurrentPosition(), pcnt);
}

void stop_stepper_motor(void) {
    if (!stepper) return;
    stepper->forceStop();
}
