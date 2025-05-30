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
    led_init();         // Initialize LED strip
    update_led_state_noHandle(STARTING); // Set initial LED state

    bsp_i2c_init();     // I2C initialization
    pca9557_init();     // IO expander initialization
    qmi8658_init();     // QMI8658 initialization

    // szp_setup();

    motor_task_init();  // Initialize motor task
    servo_init();
    button_init();
    stepper_motor_task_init(); // Initialize stepper motor task

    home_stepper_motor();
    set_stepper_pos(1000);

    update_led_state_noHandle(MACHINE_IDLE); // Set LED state to IDLE

    uart_init();
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