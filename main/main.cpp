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
#include "app_motors.h"
#include "debugHelper.cpp"




void setup(){
    bsp_i2c_init();  // I2C initialization
    pca9557_init();  // IO expander initialization
    bsp_lvgl_start(); // Lvgl initialization
    install_twai_driver();
    
    if(bsp_sdcard_mount() == ESP_OK) {
        ESP_LOGI(TAG, "SD card mounted successfully");
        bsp_codec_init(); // Codec initialization
        mp3_player_init();
    } else {
        ui_init(); // Initialize UI without SD card
        ESP_LOGE(TAG, "Failed to mount SD card");
    }
    button_init(); // Initialize button with no callback function
    start_twai_receive_task();
    motor_task_init(); // Initialize motor task
}

extern "C" void app_main(void)
{
    setup(); // Call the setup function to initialize the system

    while(1){
        // Main loop can be used for other tasks or to keep the program running
        debugLoggingTask(NULL); // Call the debug logging task
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 1 second
    }

}