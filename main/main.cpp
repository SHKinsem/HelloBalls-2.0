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


void setup(){
    bsp_i2c_init();  // I2C initialization
    pca9557_init();  // IO expander initialization
    bsp_lvgl_start(); // Lvgl initialization
    install_twai_driver();
    
    if(bsp_sdcard_mount() == ESP_OK) {
        ESP_LOGI(TAG, "SD card mounted successfully");
        // bsp_codec_init(); // Codec initialization
        // mp3_player_init();
    } else {
        ui_init(); // Initialize UI without SD card
        ESP_LOGE(TAG, "Failed to mount SD card");
        ESP_LOGE(TAG, "Entering motor Debug mode");
    }
    
    // Initialize servo on GPIO0
    // servo_init();
    
    // Initialize button with servo control function
    // button_init();
    
    motor_task_init(); // Initialize motor task
}

can_channel_t can_channel(0, TWAI_TX_PIN, TWAI_RX_PIN); // Create a CAN channel instance

extern "C" void app_main(void)
{
    // setup(); // Call the setup function to initialize the system
    // uart_init();
    dm3519_t test_motor(1); // Create an instance of the motor
    test_motor.setPIDParameters(20.0, 0.007, 0.005, 0.1, 0.1, 5000.0, -5000.0); // Set PID parameters
    can_channel.reg_motor(&test_motor); // Register the motor with the CAN channel
    can_channel.start(); // Start the CAN channel
    test_motor.setTargetSpeed(1000); // Set target speed

    while(1){
        test_motor.enable(); // Enable the motor
        vTaskDelay(pdMS_TO_TICKS(1000));
        test_motor.disable(); // Disable the motor
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}