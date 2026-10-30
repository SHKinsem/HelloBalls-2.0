#include "robot_params.h"

#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "app_motors.h"

#define TAG "PARAMS"
#define NVS_NAMESPACE "hb_params"
#define NVS_KEY "params"
// Bump when robot_params_t changes so an old blob is ignored instead of misread
#define PARAMS_VERSION 1

#ifndef CONFIG_HELLOBALLS_BLE_MAX_WHEEL_RPM
#define CONFIG_HELLOBALLS_BLE_MAX_WHEEL_RPM 3000
#endif
#ifndef CONFIG_HELLOBALLS_BLE_MAX_SHOOT_SPEED
#define CONFIG_HELLOBALLS_BLE_MAX_SHOOT_SPEED 6000
#endif

robot_params_t g_params;

static const robot_params_t defaults = {
    .max_wheel_rpm = CONFIG_HELLOBALLS_BLE_MAX_WHEEL_RPM,
    .max_shoot_speed = CONFIG_HELLOBALLS_BLE_MAX_SHOOT_SPEED,
    .tilt_min_deg = -5,
    .tilt_max_deg = 35,
    .pid = {
        {20.0f, 0.007f, 0.005f, 0.1f, 0.1f, 5000.0f, -5000.0f},  // friction wheel 1
        {20.0f, 0.007f, 0.005f, 0.1f, 0.1f, 5000.0f, -5000.0f},  // friction wheel 2
        {25.0f, 0.03f, 0.5f, 0.1f, 0.1f, 3000.0f, -3000.0f},     // left wheel
        {25.0f, 0.03f, 0.5f, 0.1f, 0.1f, 3000.0f, -3000.0f},     // right wheel
        {50.0f, 0.0001f, 80.0f, 1.0f, 0.5f, 4000.0f, -4000.0f},  // loader angle
        {25.0f, 0.02f, 15.0f, 0.1f, 1.0f, 5000.0f, -5000.0f},    // loader speed
    },
    .servo_origin = {90.0f, 94.0f},
    .search_friction_speed = -800,
    .search_stepper_pos = 1150,
    .search_tilt_down_deg = -20.0f,
    .search_tilt_hold_deg = -5.0f,
    .search_tilt_ms = 1000,
    .pickup_friction_speed = -300,
    .pickup_tilt_deg = 30.0f,
    .pickup_tilt_ms = 1000,
    .pickup_loader1_deg = 20.0f,
    .pickup_loader1_ms = 500,
    .pickup_loader2_deg = -110.0f,
    .pickup_loader2_ms = 1000,
    .shoot_stepper_pos = 1600,
    .shoot_spinup_ms = 2000,
    .shoot_loader1_deg = 135.0f,
    .shoot_loader1_ms = 1000,
    .shoot_loader2_deg = -45.0f,
    .shoot_loader2_ms = 500,
    .debug_max_rpm = 500,
};

typedef struct {
    uint32_t version;
    uint32_t size;
    robot_params_t params;
} stored_params_t;

static esp_err_t ensure_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

void robot_params_reset(void)
{
    g_params = defaults;
}

void robot_params_init(void)
{
    robot_params_reset();
    if (ensure_nvs() != ESP_OK) {
        ESP_LOGW(TAG, "NVS unavailable, using defaults");
        return;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "No saved parameters, using defaults");
        return;
    }
    stored_params_t stored;
    size_t len = sizeof(stored);
    esp_err_t err = nvs_get_blob(h, NVS_KEY, &stored, &len);
    nvs_close(h);
    if (err == ESP_OK && len == sizeof(stored) && stored.version == PARAMS_VERSION &&
        stored.size == sizeof(robot_params_t)) {
        g_params = stored.params;
        ESP_LOGI(TAG, "Loaded saved parameters");
    } else {
        ESP_LOGW(TAG, "Saved parameters missing or from another firmware, using defaults");
    }
}

esp_err_t robot_params_save(void)
{
    stored_params_t stored = {};
    stored.version = PARAMS_VERSION;
    stored.size = sizeof(robot_params_t);
    stored.params = g_params;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, NVS_KEY, &stored, sizeof(stored));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGI(TAG, "Save parameters: %s", esp_err_to_name(err));
    return err;
}

void robot_params_apply(void)
{
    for (int i = 0; i < PID_COUNT; i++) {
        app_motor_set_pid((uint8_t)(i + 1), g_params.pid[i]);
    }
    set_servo_origins(g_params.servo_origin[0], g_params.servo_origin[1]);
}
