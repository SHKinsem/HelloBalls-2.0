/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a master node in a TWAI network. The master
 * node is responsible for initiating and stopping the transfer of data messages.
 * The example will execute multiple iterations, with each iteration the master
 * node will do the following:
 * 1) Start the TWAI driver
 * 2) Repeatedly send ping messages until a ping response from slave is received
 * 3) Send start command to slave and receive data messages from slave
 * 4) Send stop command to slave and wait for stop response from slave
 * 5) Stop the TWAI driver
 */

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
    led_init(); // Initialize LED strip
    update_led_state_noHandle(STARTING); // Set initial LED state

    bsp_i2c_init();  // I2C initialization
    pca9557_init();  // IO expander initialization
    qmi8658_init(); // QMI8658 initialization

    // szp_setup(); // Call the setup function to initialize components

    motor_task_init(); // Initialize motor task
    servo_init();
    button_init();
    update_led_state_noHandle(MACHINE_IDLE); // Set LED state to IDLE

    uart_init();

}


extern "C" void app_main(void)
{
    setup(); // Call setup function to initialize components

    while(1){
        // qmi8658_fetch_angleFromAcc(&qmi8658_data); // Fetch angle data from QMI8658
        // ESP_LOGI(TAG, "AngleX: %f, AngleY: %f, AngleZ: %f", qmi8658_data.AngleX, qmi8658_data.AngleY, qmi8658_data.AngleZ); // Log angle data
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}