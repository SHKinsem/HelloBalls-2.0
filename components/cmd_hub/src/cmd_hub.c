#include "cmd_hub.h"

#include <string.h>
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "CMD_HUB";

#define HOST_IDLE 0  // host_state_t value, kept local so the hub does not depend on serial

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_cmd_sem;    // given for every accepted command
static SemaphoreHandle_t s_state_sem;  // given when the effective host state changes

static robot_cmd_t s_current;
static int64_t s_last_us[CMD_SRC_COUNT];
static bool s_takeover;
static bool s_estop;
static uint8_t s_host_state = HOST_IDLE;

static bool is_fresh(cmd_source_t src, int64_t now)
{
    return s_last_us[src] != 0 && (now - s_last_us[src]) < (int64_t)CMD_HUB_SOURCE_TIMEOUT_MS * 1000;
}

// Caller holds s_lock. Returns true when the state semaphore must be given.
static bool set_state_locked(uint8_t state)
{
    if (state == s_host_state) {
        return false;
    }
    s_host_state = state;
    return true;
}

// Caller holds s_lock: stop everything and fall back to idle
static bool force_idle_locked(void)
{
    memset(&s_current, 0, sizeof(s_current));
    return set_state_locked(HOST_IDLE);
}

static void signal_state(bool changed)
{
    if (changed) {
        xSemaphoreGive(s_state_sem);
    }
}

void cmd_hub_init(void)
{
    if (s_cmd_sem == NULL) {
        s_cmd_sem = xSemaphoreCreateBinary();
        s_state_sem = xSemaphoreCreateBinary();
    }
}

void cmd_hub_submit(cmd_source_t src, const robot_cmd_t *cmd)
{
    if (cmd == NULL || src >= CMD_SRC_COUNT) {
        return;
    }
    int64_t now = esp_timer_get_time();
    bool state_changed = false;
    bool accepted = false;

    taskENTER_CRITICAL(&s_lock);
    s_last_us[src] = now;

    // BLE went quiet while in control: stop, then hand control back to the host
    if (s_takeover && !is_fresh(CMD_SRC_BLE, now)) {
        s_takeover = false;
        state_changed |= force_idle_locked();
    }

    cmd_source_t active = s_takeover ? CMD_SRC_BLE : CMD_SRC_SERIAL;
    if (src == active && !s_estop) {
        s_current = *cmd;
        state_changed |= set_state_locked(cmd->host_state);
        accepted = true;
    }
    taskEXIT_CRITICAL(&s_lock);

    signal_state(state_changed);
    if (accepted) {
        xSemaphoreGive(s_cmd_sem);
    }
}

void cmd_hub_request_takeover(bool takeover)
{
    bool state_changed = false;
    taskENTER_CRITICAL(&s_lock);
    if (s_takeover != takeover) {
        s_takeover = takeover;
        state_changed = force_idle_locked();
    }
    taskEXIT_CRITICAL(&s_lock);

    if (state_changed || takeover) {
        ESP_LOGI(TAG, "Control: %s", takeover ? "BLE remote" : "serial host");
    }
    signal_state(state_changed);
}

void cmd_hub_source_lost(cmd_source_t src)
{
    if (src >= CMD_SRC_COUNT) {
        return;
    }
    bool state_changed = false;
    taskENTER_CRITICAL(&s_lock);
    s_last_us[src] = 0;
    bool was_active = (src == CMD_SRC_BLE) ? s_takeover : !s_takeover;
    if (src == CMD_SRC_BLE) {
        s_takeover = false;
    }
    if (was_active) {
        state_changed = force_idle_locked();
    }
    taskEXIT_CRITICAL(&s_lock);
    signal_state(state_changed);
}

void cmd_hub_set_estop(bool estop)
{
    bool state_changed = false;
    taskENTER_CRITICAL(&s_lock);
    s_estop = estop;
    if (estop) {
        state_changed = force_idle_locked();
    }
    taskEXIT_CRITICAL(&s_lock);
    ESP_LOGW(TAG, "Emergency stop %s", estop ? "LATCHED" : "cleared");
    signal_state(state_changed);
}

bool cmd_hub_wait_command(robot_cmd_t *out, TickType_t timeout)
{
    if (xSemaphoreTake(s_cmd_sem, timeout) != pdTRUE) {
        return false;
    }
    bool ok;
    taskENTER_CRITICAL(&s_lock);
    ok = !s_estop;
    if (ok && out != NULL) {
        *out = s_current;
    }
    taskEXIT_CRITICAL(&s_lock);
    return ok;
}

uint8_t cmd_hub_wait_state_change(TickType_t timeout, bool *changed)
{
    bool got = xSemaphoreTake(s_state_sem, timeout) == pdTRUE;
    if (changed != NULL) {
        *changed = got;
    }
    return cmd_hub_host_state();
}

uint8_t cmd_hub_host_state(void)
{
    uint8_t state;
    taskENTER_CRITICAL(&s_lock);
    state = s_host_state;
    taskEXIT_CRITICAL(&s_lock);
    return state;
}

void cmd_hub_get_status(cmd_hub_status_t *out)
{
    if (out == NULL) {
        return;
    }
    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_lock);
    out->takeover = s_takeover;
    out->active = s_takeover ? CMD_SRC_BLE : CMD_SRC_SERIAL;
    out->estop = s_estop;
    for (int i = 0; i < CMD_SRC_COUNT; i++) {
        out->fresh[i] = is_fresh((cmd_source_t)i, now);
    }
    out->host_state = s_host_state;
    taskEXIT_CRITICAL(&s_lock);
}
