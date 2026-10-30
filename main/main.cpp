#define TAG "app_main"
#define GLOBAL_IQ 15    // IQ math library global IQ value
#include "board_pins.h"
#include "esp32_s3_szp.h"
#include "app_ui.h"
#include "app_button.h"
#include "app_twai.h"
#include "ui.h"
// #include "app_motors.h"
#include "app_motors.h"
#include "debugger.h"
#include "serial.h"
#include "stepper_motors.h"
#include "leds.h"
#include "state_machine.h"
#include "cmd_hub.h"
#include "remote_link.h"
#include "robot_params.h"
#include "health.h"

void szp_setup(){
    if(bsp_sdcard_mount() == ESP_OK) {
        ESP_LOGI(TAG, "SD card mounted successfully");
        // bsp_codec_init(); // Codec initialization
        // mp3_player_init();
    } else {
        bsp_lvgl_start(); // Lvgl initialization
        ui_init(); // Initialize UI without SD card
        ESP_LOGE(TAG, "Failed to mount SD card");
        ESP_LOGE(TAG, "Entering motor Debug mode");
    }
}

void setup(){
    robot_params_init(); // Tunable parameters (BLE object dictionary), saved ones from NVS
    cmd_hub_init();     // Command hub shared by the serial host and the BLE remote
    health_set_stage(HEALTH_STAGE_BLE);
    remote_link_init(); // BLE first, so the phone can see the robot even if a later step hangs
    led_init();         // Initialize LED strip
    update_led_state_noHandle(STARTING); // Set initial LED state

    health_set_stage(HEALTH_STAGE_IMU);
    bsp_i2c_init();     // I2C initialization
    pca9557_init();     // IO expander initialization
    qmi8658_init();     // QMI8658 initialization (waits until the chip answers)

    // szp_setup();

    health_set_stage(HEALTH_STAGE_CAN);
    motor_task_init();  // Initialize motor task
    health_set_stage(HEALTH_STAGE_SERVO);
    servo_init();
    robot_params_apply(); // Saved PID gains and servo origins
    button_init();
    health_set_stage(HEALTH_STAGE_STEPPER);
    stepper_motor_task_init(); // Initialize stepper motor task
    remote_link_set_hw_ready(); // Motors exist now, BLE e-stop may drive them

    health_set_stage(HEALTH_STAGE_HOMING);
    if (home_stepper_motor()) { // Times out instead of blocking boot; retried on the next state change
        set_stepper_pos(1000);
    }

    update_led_state_noHandle(MACHINE_IDLE); // Set LED state to IDLE

    health_set_stage(HEALTH_STAGE_SERIAL);
    uart_init();
    health_set_stage(HEALTH_STAGE_READY);
}


extern "C" void app_main(void)
{
    setup();
    state_machine_init();

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before starting tasks

    while(1){
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}