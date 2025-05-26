#pragma once

#include "bldc_motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef CAN_CHANNEL_H
#define CAN_CHANNEL_H

/*
* This class represents a CAN channel for motor communication.
* It handles the registration and management of motors within the CAN network.
*/
class can_channel_t
{
private:
    uint8_t channel_id; // Channel ID for the CAN channel
    uint8_t motorCount;
    base_motor_t* motors[8];
    twai_handle_t twai_handle;
    twai_general_config_t g_config;
    twai_timing_config_t t_config;
    twai_filter_config_t f_config;
    twai_message_t tx_message[2];
    int motorUpdateFreq;
    bool twai_running;
    TaskHandle_t alert_task_handle;
    uint32_t tx_failed_count;
    
    // Added variables for tracking message reception
    TickType_t last_rx_time;          // Timestamp of last received message
    bool tx_active;                   // Flag to indicate if TX should be active
    uint32_t rx_timeout_ms;           // Timeout period in ms after which TX will stop if no messages received

    bool stop_flag; // Flag to indicate if the TWAI driver should stop
    
    void rx_task(void* arg);
    void tx_task(void* arg);
    void alert_task(void* arg);
    bool handle_twai_alert(uint32_t alerts);
    bool recover_from_bus_off();
    bool recover_from_error_passive();
    bool recover_from_rx_queue_full();
    void log_alert_flags(uint32_t flags);
    void updateMotorControlOutput();

public:
    can_channel_t(uint8_t channel_id, gpio_num_t tx_io, gpio_num_t rx_io);
    ~can_channel_t();
    can_channel_t(const can_channel_t&) = delete; // Disable copy constructor

    void reg_motor(base_motor_t* motor);
    base_motor_t** getMotorList();
    void setChannelId(uint8_t channel_id) { this->channel_id = channel_id; }
    uint8_t getChannelId() const { return this->channel_id; }
    void setMotorCount(uint8_t motorCount) { this->motorCount = motorCount; }
    void sendMessage(twai_message_t* msg);

    void start();
    void resume();
    void pause();
    void stop();

};

#endif // CAN_CHANNEL_H
