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


t_sQMI8658 qmi8658_data; // QMI8658 data structure

extern m2006_t loaderMotor; // Loader motor instance

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
    update_led_state_noHandle(MACHINE_IDLE); // Set LED state to IDLE

    uart_init();
}


extern "C" void app_main(void)
{
    setup();

    while(1){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}