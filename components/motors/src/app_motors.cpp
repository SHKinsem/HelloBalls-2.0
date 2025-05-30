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
#define SERVO_2_MIDDLE_ANGLE 91.0f // Middle position for servo 2
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
void tilt_servos(float angle) {
    // Clamp angle to valid range
    if (angle > MAX_TILT_ANGLE) angle = MAX_TILT_ANGLE;
    if (angle < MIN_TILT_ANGLE) angle = MIN_TILT_ANGLE;

    // Tilt both servos by the specified angle
    servo1.moveRelativeToOrigin(-angle, 80.0f); // Move servo 1 relative to origin
    servo2.moveRelativeToOrigin(angle, 80.0f); // Move servo 2 relative to origin
}

void disable_servos() {
    servo1.disable();
    servo2.disable();
}

void enable_servos() {
    servo1.enable();
    servo2.enable();
}

can_channel_t can_channel(0, TWAI_TX_PIN, TWAI_RX_PIN); // Create a CAN channel instance
dm3519_t test_motor(1); // Create an instance of the motor

m3508_t frictionWheel_1(1);
m3508_t frictionWheel_2(2);
m3508_t wheelMotor_1(3);
m3508_t wheelMotor_2(4);
m2006_t loaderMotor(5);

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
    // Reset the loader motor to its origin position
    loaderMotor.resetCounter(); // Reset the loop counter
    loaderMotor.setTarget(0.0f);
    loader_origin_angle = 0.0f; // Reset the global loader origin angle
}

void motor_task_init(){

    serial_rx_semaphore = xSemaphoreCreateBinary();
    state_change_semaphore = xSemaphoreCreateBinary();

    frictionWheel_1.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0);
    frictionWheel_2.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0);
    wheelMotor_1.setPIDParameters(25.0, 0.03, 0.5, 0.1, 0.1, 2000.0, -2000.0);
    wheelMotor_2.setPIDParameters(25.0, 0.03, 0.5, 0.1, 0.1, 2000.0, -2000.0);

    loaderMotor.initAnglePID();
    controller_t<int16_t>* speed_controller = new controller_t<int16_t>(loaderMotor.getRawSpeedPtr());
    loaderMotor.setNextController(speed_controller);

    loaderMotor.setPIDParameters(100.0f, 0.0001f, 80.0f, 1.0f, 0.5f, 4000.0f, -4000.0f);
    speed_controller->setPIDParameters(25.0f, 0.02f, 15.0f, 0.1f, 1.0f, 5000.0f, -5000.0f);

    can_channel.reg_motor(&test_motor);
    can_channel.reg_motor(&frictionWheel_1);
    can_channel.reg_motor(&frictionWheel_2);
    can_channel.reg_motor(&wheelMotor_1);
    can_channel.reg_motor(&wheelMotor_2);
    can_channel.reg_motor(&loaderMotor);
    can_channel.start();
}
