#include "health.h"

#include <stdio.h>
#include <string.h>
#include "app_motors.h"
#include "ble_remote.h"
#include "cmd_hub.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "stepper_motors.h"

#define TAG "HEALTH"

// A stage that takes longer than this is reported as stuck
#define STUCK_MS 3000
// Motors get this long after CAN starts before missing feedback counts
#define MOTOR_GRACE_MS 1500

static volatile health_stage_t s_stage = HEALTH_STAGE_BOOT;
static volatile int64_t s_stage_since_us;
static volatile int64_t s_can_since_us;
static volatile uint32_t s_faults;
static uint32_t s_adv_faults = UINT32_MAX;
static health_stage_t s_adv_stage = HEALTH_STAGE_READY;

static const char *const STAGE_TEXT[] = {
    "启动中", "启动蓝牙", "初始化 IMU", "启动 CAN 电机", "初始化舵机", "初始化步进电机", "步进回零", "启动串口", "就绪",
};
static const char *const MOTOR_TEXT[] = {"摩擦轮 1", "摩擦轮 2", "左轮", "右轮", "拨球"};

static void update_advert(void)
{
    health_stage_t stage = s_stage;
    uint32_t faults = s_faults;
    if (stage == s_adv_stage && faults == s_adv_faults) {
        return;
    }
    s_adv_stage = stage;
    s_adv_faults = faults;
#if CONFIG_HELLOBALLS_BLE_REMOTE
    uint8_t d[HEALTH_MFG_LEN] = {0xFF, 0xFF, 'H', 'B', 1, (uint8_t)stage,
                                 (uint8_t)faults, (uint8_t)(faults >> 8), (uint8_t)(faults >> 16), (uint8_t)(faults >> 24)};
    ble_remote_set_mfg_data(d, sizeof(d));
#endif
    char reason[160];
    health_reason(reason, sizeof(reason));
    ESP_LOGI(TAG, "stage %d faults 0x%08lx: %s", (int)stage, (unsigned long)faults, reason);
}

void health_set_stage(health_stage_t stage)
{
    int64_t now = esp_timer_get_time();
    s_stage = stage;
    s_stage_since_us = now;
    if (stage > HEALTH_STAGE_CAN && s_can_since_us == 0) {
        s_can_since_us = now;
    }
    health_refresh();
}

health_stage_t health_stage(void)
{
    return s_stage;
}

uint32_t health_faults(void)
{
    return s_faults;
}

void health_refresh(void)
{
    int64_t now = esp_timer_get_time();
    health_stage_t stage = s_stage;
    uint32_t f = 0;

    // IMU init waits forever for the chip; say so instead of just sitting in the stage
    if (stage == HEALTH_STAGE_IMU && now - s_stage_since_us > (int64_t)STUCK_MS * 1000) {
        f |= HEALTH_F_IMU;
    }
    if (stage > HEALTH_STAGE_CAN) {
        if (!app_can_running()) {
            f |= HEALTH_F_CAN;
        } else if (now - s_can_since_us > (int64_t)MOTOR_GRACE_MS * 1000) {
            for (uint8_t id = APP_MOTOR_ID_FIRST; id <= APP_MOTOR_ID_LAST; id++) {
                app_motor_info_t info;
                if (app_motor_get_info(id, &info) && !info.online) {
                    f |= HEALTH_F_MOTOR(id);
                }
            }
        }
    }
    if (stage > HEALTH_STAGE_HOMING && !stepper_is_homed()) {
        f |= HEALTH_F_HOMING;
    }
    cmd_hub_status_t hub;
    cmd_hub_get_status(&hub);
    if (stage == HEALTH_STAGE_READY && !hub.fresh[CMD_SRC_SERIAL]) {
        f |= HEALTH_F_SERIAL;
    }
    if (hub.estop) {
        f |= HEALTH_F_ESTOP;
    }
    s_faults = f;
    update_advert();
}

static void append(char *buf, size_t cap, size_t *pos, const char *text)
{
    if (*pos >= cap) {
        return;
    }
    int n = snprintf(buf + *pos, cap - *pos, "%s%s", *pos ? "；" : "", text);
    if (n > 0) {
        *pos += (size_t)n < cap - *pos ? (size_t)n : cap - *pos - 1;
    }
}

void health_reason(char *buf, size_t cap)
{
    if (cap == 0) {
        return;
    }
    buf[0] = '\0';
    size_t pos = 0;
    health_stage_t stage = s_stage;
    uint32_t f = s_faults;
    char line[48];
    if (stage != HEALTH_STAGE_READY) {
        bool stuck = esp_timer_get_time() - s_stage_since_us > (int64_t)STUCK_MS * 1000;
        snprintf(line, sizeof(line), "%s%s", stuck ? "卡在" : "正在", STAGE_TEXT[stage]);
        append(buf, cap, &pos, line);
    }
    if (f & HEALTH_F_IMU) append(buf, cap, &pos, "IMU 无响应");
    if (f & HEALTH_F_CAN) append(buf, cap, &pos, "CAN 驱动没有启动");
    for (uint8_t id = APP_MOTOR_ID_FIRST; id <= APP_MOTOR_ID_LAST; id++) {
        if (f & HEALTH_F_MOTOR(id)) {
            snprintf(line, sizeof(line), "%s（CAN %u）无反馈", MOTOR_TEXT[id - 1], id);
            append(buf, cap, &pos, line);
        }
    }
    if (f & HEALTH_F_HOMING) append(buf, cap, &pos, "步进回零超时，限位开关没有触发");
    if (f & HEALTH_F_SERIAL) append(buf, cap, &pos, "上位机串口没有数据");
    if (f & HEALTH_F_ESTOP) append(buf, cap, &pos, "急停中");
    if (pos == 0) append(buf, cap, &pos, "正常");
}
