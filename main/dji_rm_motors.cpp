#include "dji_rm_motors.h"

can_channel_t::can_channel_t()
{
    channel_id = 0;
    baud_rate = CAN_SPEED;
    gpio_tx = GPIO_NUM_21;
    gpio_rx = GPIO_NUM_22;
    t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps
    f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_tx, gpio_rx, TWAI_MODE_NORMAL);
}