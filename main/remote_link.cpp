/* Glue between the BLE remote (ble_remote component) and the command hub. */
#include "remote_link.h"
#include "sdkconfig.h"

#if CONFIG_HELLOBALLS_BLE_REMOTE

#include "ble_remote.h"
#include "cmd_hub.h"
#include "serial.h"
#include "app_motors.h"
#include "stepper_motors.h"
#include "od_server.h"
#include "robot_params.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "REMOTE_LINK"
#define STATUS_PERIOD_MS 100

static volatile uint8_t last_seq = 0;
static volatile bool hw_ready = false; // BLE starts before the motor drivers exist

static int16_t clamp16(int32_t v, int32_t lo, int32_t hi) {
    return (int16_t)(v < lo ? lo : (v > hi ? hi : v));
}

static void on_command(const ble_remote_cmd_t *in, void *ctx) {
    robot_cmd_t cmd;
    cmd.host_state = in->state <= HOST_ERROR ? in->state : (uint8_t)HOST_IDLE;
    // Limits are object dictionary parameters 0x3000 (defaults from Kconfig)
    const int32_t wheel = g_params.max_wheel_rpm, shoot = g_params.max_shoot_speed;
    cmd.wheel1_speed = clamp16(in->wheel1_rpm, -wheel, wheel);
    cmd.wheel2_speed = clamp16(in->wheel2_rpm, -wheel, wheel);
    cmd.tilt_angle = clamp16(in->tilt_angle, g_params.tilt_min_deg, g_params.tilt_max_deg);
    cmd.shoot_speed = clamp16(in->shoot_speed, -shoot, shoot);
    last_seq = in->seq;

    cmd_hub_request_takeover((in->flags & BLE_REMOTE_FLAG_TAKEOVER) != 0);
    cmd_hub_submit(CMD_SRC_BLE, &cmd);
}

void remote_link_emergency_stop(void) {
    cmd_hub_set_estop(true);
    if (!hw_ready) return; // Latched in cmd_hub; nothing is driving yet
    // Cut the motors now instead of waiting for the watchdog timeout
    motorStateHelper(false);
    set_wheel_motors_speed(0, 0);
    set_friction_wheels_speed(0);
    stop_stepper_motor();
    disable_servos();
}

static void on_action(const ble_remote_action_t *action, void *ctx) {
    switch (action->action) {
    case BLE_REMOTE_ACTION_ESTOP:
        remote_link_emergency_stop();
        break;
    case BLE_REMOTE_ACTION_CLEAR_ESTOP:
        cmd_hub_set_estop(false);
        break;
    default:
        ESP_LOGW(TAG, "Unknown action 0x%02x", action->action);
        break;
    }
}

static void on_sdo(const uint8_t *data, size_t len, void *ctx) {
    od_server_on_sdo(data, len);
}

static void on_connection(bool connected, void *ctx) {
    od_server_on_connection(connected);
    if (!connected) {
        cmd_hub_source_lost(CMD_SRC_BLE); // stop immediately if the remote was driving
    }
}

static void status_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        cmd_hub_status_t hub;
        cmd_hub_get_status(&hub);
        int32_t w1 = 0, w2 = 0;
        get_tx_wheel_speed_feedback(&w1, &w2);

        ble_remote_status_t st = {};
        st.mcu_state = (uint8_t)*getMcuState();
        st.host_state = hub.host_state;
        if (hub.takeover && hub.fresh[CMD_SRC_BLE]) {
            st.active_src = BLE_REMOTE_SRC_BLE;
        } else if (!hub.takeover && hub.fresh[CMD_SRC_SERIAL]) {
            st.active_src = BLE_REMOTE_SRC_SERIAL;
        } else {
            st.active_src = BLE_REMOTE_SRC_NONE;
        }
        st.flags = (hub.estop ? BLE_REMOTE_STATUS_ESTOP : 0) | (hub.takeover ? BLE_REMOTE_STATUS_TAKEOVER : 0);
        st.last_seq = last_seq;
        st.wheel1_rpm = clamp16(w1, INT16_MIN, INT16_MAX);
        st.wheel2_rpm = clamp16(w2, INT16_MIN, INT16_MAX);
        st.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
        ble_remote_notify_status(&st);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STATUS_PERIOD_MS));
    }
}

void remote_link_init(void) {
    ble_remote_config_t cfg = {};
    cfg.on_command = on_command;
    cfg.on_action = on_action;
    cfg.on_sdo = on_sdo;
    cfg.on_connection = on_connection;
    cfg.fw_version = esp_app_get_description()->version;

    esp_err_t err = ble_remote_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE remote disabled: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "BLE remote ready: %s", ble_remote_device_name());
    od_server_init();
    xTaskCreate(status_task, "remote_status", 3072, NULL, 2, NULL);
}

void remote_link_set_hw_ready(void) {
    hw_ready = true;
}

bool remote_link_hw_ready(void) {
    return hw_ready;
}

#else

void remote_link_init(void) {}
void remote_link_set_hw_ready(void) {}
bool remote_link_hw_ready(void) { return false; }
void remote_link_emergency_stop(void) {}

#endif // CONFIG_HELLOBALLS_BLE_REMOTE
