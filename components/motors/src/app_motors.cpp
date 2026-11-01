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
// #include "app_twai.h"
#include "app_motors.h"
#include "serial.h"
#include <algorithm>  // For std::min and std::max
#include "servo.h"

#define TAG "MOTORS"

#define SERVO_1_MIDDLE_ANGLE 90.0f // Middle position for servo 1
#define SERVO_2_MIDDLE_ANGLE 94.0f // Middle position for servo 2
#define MAX_SPEED 5000 // Maximum speed for motors



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


bool servo_state = true;
// Toggle servo position - helper function for C code
void toggle_servo(void) {
    if(servo_state) {
        servo1.moveRelativeToOrigin(-30.0f, 100.0f); // Move servo 1 to 30 degrees relative to origin
        servo2.moveRelativeToOrigin(30.0f, 100.0f); // Move servo 2 to -30 degrees relative to origin
    } else {
        servo1.moveRelativeToOrigin(3.0f, 100.0f); 
        servo2.moveRelativeToOrigin(-3.0f, 100.0f); 
    }
    servo_state = !servo_state;
}

#define MAX_TILT_ANGLE 40.0f // Maximum tilt angle for servos
#define MIN_TILT_ANGLE -10.0f // Minimum tilt angle for servos
static float last_tilt_angle = 0.0f;
void tilt_servos(float angle) {
    // Clamp angle to valid range
    if (angle > MAX_TILT_ANGLE) angle = MAX_TILT_ANGLE;
    if (angle < MIN_TILT_ANGLE) angle = MIN_TILT_ANGLE;
    last_tilt_angle = angle;

    // Tilt both servos by the specified angle
    servo1.moveRelativeToOrigin(angle, 80.0f); // Move servo 1 relative to origin
    servo2.moveRelativeToOrigin(-angle, 80.0f); // Move servo 2 relative to origin
}

void disable_servos() {
    servo1.disable();
    servo2.disable();
}

void enable_servos() {
    servo1.enable();
    servo2.enable();
}

float get_tilt_angle(void) {
    return last_tilt_angle;
}

bool servos_enabled(void) {
    return servo1.isEnabled() && servo2.isEnabled();
}

void set_servo_origins(float servo1_deg, float servo2_deg) {
    servo1.setOriginAngle(servo1_deg);
    servo2.setOriginAngle(servo2_deg);
}

void move_servos_to_origin(void) {
    if (!servos_enabled()) return;
    servo1.reset();
    servo2.reset();
}

void set_servo_offsets(float servo1_deg, float servo2_deg) {
    enable_servos();
    servo1.moveRelativeToOrigin(servo1_deg);
    servo2.moveRelativeToOrigin(servo2_deg);
}

static StaticSemaphore_t s_cmd_mutex_buf;
static SemaphoreHandle_t s_cmd_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_cmd_mutex_buf);

void app_motor_cmd_lock(void) {
    xSemaphoreTakeRecursive(s_cmd_mutex, portMAX_DELAY);
}

void app_motor_cmd_unlock(void) {
    xSemaphoreGiveRecursive(s_cmd_mutex);
}

can_channel_t can_channel(0, TWAI_TX_PIN, TWAI_RX_PIN); // Create a CAN channel instance
dm3519_t test_motor(1); // Create an instance of the motor

m3508_t frictionWheel_1(1);
m3508_t frictionWheel_2(2);
m3508_t wheelMotor_1(3);
m3508_t wheelMotor_2(4);
m2006_t loaderMotor(5);

static void wheel_speed_feedback_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        /*
         * M3508 feedback speed is signed RPM. wheelMotor_2 is installed in
         * the opposite direction, matching the sign conversion used when
         * commands are applied in set_wheel_motors_speed().
         */
        set_tx_wheel_speed_feedback(
            (int32_t)wheelMotor_1.getRawSpeed(),
            -(int32_t)wheelMotor_2.getRawSpeed());
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

void motorStateHelper(bool state) {
    if (state) {
        frictionWheel_1.enable();
        frictionWheel_2.enable();
        wheelMotor_1.enable();
        wheelMotor_2.enable();
        loaderMotor.enable();
    } else {
        frictionWheel_1.disable();
        frictionWheel_2.disable();
        wheelMotor_1.disable();
        wheelMotor_2.disable();
        loaderMotor.disable();
    }
}

void set_friction_wheels_speed(const int16_t& speed) {
    if(speed == 0) {
        frictionWheel_1.disable();
        frictionWheel_2.disable();
        return; // Stop motors if speed is zero
    }
    frictionWheel_1.setTarget(-speed);
    frictionWheel_2.setTarget(speed);
}

void set_wheel_motors_speed(const int16_t& speed1, const int16_t& speed2) {
    wheelMotor_1.setTarget(speed1);
    wheelMotor_2.setTarget(-speed2);
}


static controller_t<int16_t>* loader_speed_controller = nullptr;

static base_motor_t* motor_by_id(uint8_t can_id) {
    switch (can_id) {
    case 1: return &frictionWheel_1;
    case 2: return &frictionWheel_2;
    case 3: return &wheelMotor_1;
    case 4: return &wheelMotor_2;
    case 5: return &loaderMotor;
    default: return nullptr;
    }
}

bool app_motor_get_info(uint8_t can_id, app_motor_info_t *out) {
    base_motor_t* m = motor_by_id(can_id);
    if (!m || !out) return false;
    out->speed_rpm = m->getRawSpeed();
    out->target = m->getTarget();
    out->current_a = m->getCurrent();
    out->temperature_c = m->getTemperature();
    out->angle_deg = m->getAngle();
    out->status = m->getStatus();
    out->enabled = m->isEnabled();
    out->online = m->isOnline(APP_MOTOR_ONLINE_MS);
    return true;
}

bool app_can_running(void) {
    return can_channel.isRunning();
}

float app_loader_shaft_angle(void) {
    return loaderMotor.getShaftAnglePtr() ? loaderMotor.getShaftAngle() : 0.0f;
}

float app_loader_pid_error(void) {
    return loaderMotor.getPID_Error();
}

bool app_motor_debug_drive(uint8_t can_id, float rpm) {
    if (can_id == APP_MOTOR_ID_LOADER || (can_id != 0 && !motor_by_id(can_id))) return false;
    for (uint8_t id = APP_MOTOR_ID_FIRST; id <= APP_MOTOR_ID_LAST; id++) {
        base_motor_t* m = motor_by_id(id);
        if (id == can_id) {
            m->setTarget(rpm);
            m->enable();
        } else {
            m->disable();
            if (id != APP_MOTOR_ID_LOADER) m->setTarget(0);
        }
    }
    return true;
}

bool app_motor_set_pid(uint8_t pid_id, const float g[7]) {
    if (pid_id == APP_PID_LOADER_SPEED) {
        if (!loader_speed_controller) return false;
        loader_speed_controller->setPIDParameters(g[0], g[1], g[2], g[3], g[4], g[5], g[6]);
        return true;
    }
    base_motor_t* m = motor_by_id(pid_id);
    if (!m) return false;
    m->setPIDParameters(g[0], g[1], g[2], g[3], g[4], g[5], g[6]);
    return true;
}

static float loader_origin_angle = 0.0f;

// Move loader to a specified angle
void moveLoader(float angle) {
    // Clamp angle to valid range
    angle = loaderMotor.getShaftAngle() + angle; // Adjust angle relative to origin
    if (angle < -180.0f) angle = -180.0f;
    if (angle > 180.0f) angle = 180.0f;
    if (fabs(angle) < 0.1f) {
        ESP_LOGI(TAG, "Loader angle change too small, ignoring: %.2f", angle);
        return; // Ignore small changes to prevent jitter
    }
    loaderMotor.setTarget(angle);
}

void resetLoaderOrigin() {
    // Reset the loop counter and the target together, so the 1 kHz angle loop never
    // runs one cycle with the new counter and the old target
    loaderMotor.resetOrigin();
    loader_origin_angle = 0.0f; // Reset the global loader origin angle
}

void motor_task_init(){

    frictionWheel_1.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0);
    frictionWheel_2.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0);
    wheelMotor_1.setPIDParameters(25.0, 0.03, 0.5, 0.1, 0.1, 3000.0, -3000.0);
    wheelMotor_2.setPIDParameters(25.0, 0.03, 0.5, 0.1, 0.1, 3000.0, -3000.0);

    loaderMotor.initAnglePID();
    controller_t<int16_t>* speed_controller = new controller_t<int16_t>(loaderMotor.getRawSpeedPtr());
    loaderMotor.setNextController(speed_controller);
    loader_speed_controller = speed_controller;

    loaderMotor.setPIDParameters(50.0f, 0.0001f, 80.0f, 1.0f, 0.5f, 4000.0f, -4000.0f);
    speed_controller->setPIDParameters(25.0f, 0.02f, 15.0f, 0.1f, 1.0f, 5000.0f, -5000.0f);

    can_channel.reg_motor(&test_motor);
    can_channel.reg_motor(&frictionWheel_1);
    can_channel.reg_motor(&frictionWheel_2);
    can_channel.reg_motor(&wheelMotor_1);
    can_channel.reg_motor(&wheelMotor_2);
    can_channel.reg_motor(&loaderMotor);
    can_channel.start();

    xTaskCreate(wheel_speed_feedback_task, "wheel_speed_feedback", 2048,
                NULL, 3, NULL);
}
