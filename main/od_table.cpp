#include "od_table.h"

#include "sdkconfig.h"

#if CONFIG_HELLOBALLS_BLE_REMOTE

#include <stdlib.h>
#include <string.h>
#include "app_motors.h"
#include "ble_remote.h"
#include "cmd_hub.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "health.h"
#include "motor_debug.h"
#include "remote_link.h"
#include "robot_params.h"
#include "serial.h"
#include "stepper_motors.h"

#define TAG "OD_TABLE"

#define SAVE_SIGNATURE 0x65766173u  // "save", as in CANopen 0x1010
#define LOAD_SIGNATURE 0x64616F6Cu  // "load", as in CANopen 0x1011

od_tpdo_t g_tpdo[OD_TPDO_COUNT];

// Live values copied by od_table_refresh(); entries point here
typedef struct {
    int16_t speed;
    float target;
    float current;
    float temperature;
    float angle;
    int8_t status;
    bool enabled;
    uint8_t can_id;
} motor_live_t;

static struct {
    uint8_t mcu_state;
    uint8_t host_state;
    uint8_t source;
    bool estop;
    bool takeover;
    uint32_t uptime_ms;
    uint8_t stage;
    uint32_t faults;
    motor_live_t motor[APP_MOTOR_ID_LAST];
    float loader_shaft;
    float loader_error;
    float tilt;
    bool servos_on;
    int32_t stepper_pos;
    int32_t stepper_target;
    bool endstop;
    bool stepper_running;
    bool stepper_homed;
    int16_t acc[3];
    int16_t gyr[3];
    uint32_t imu_seq;
    int16_t debug_speed;
    float debug_current;
    float debug_temperature;
} live;

static uint8_t s_reset_reason;
static char *s_desc;
static size_t s_desc_len;
static uint32_t s_desc_crc;

// ---- hooks -----------------------------------------------------------------

static uint32_t read_text(const char *s, uint32_t offset, uint8_t *buf, size_t cap, size_t *len, uint32_t *total)
{
    size_t n = s ? strlen(s) : 0;
    *total = (uint32_t)n;
    if (offset > n) {
        return OD_ABORT_LENGTH;
    }
    *len = n - offset < cap ? n - offset : cap;
    memcpy(buf, s + offset, *len);
    return OD_OK;
}

static uint32_t rd_name(const od_entry_t *, uint32_t off, uint8_t *buf, size_t cap, size_t *len, uint32_t *total)
{
    return read_text(ble_remote_device_name(), off, buf, cap, len, total);
}

static uint32_t rd_reason(const od_entry_t *, uint32_t off, uint8_t *buf, size_t cap, size_t *len, uint32_t *total)
{
    char text[200];
    health_reason(text, sizeof(text));
    return read_text(text, off, buf, cap, len, total);
}

static uint32_t rd_crash(const od_entry_t *, uint32_t off, uint8_t *buf, size_t cap, size_t *len, uint32_t *total)
{
    return read_text(health_crash_text(), off, buf, cap, len, total);
}

static uint32_t wr_clear_crash(const od_entry_t *, const uint8_t *d, size_t)
{
    if (d[0]) {
        health_clear_crash();
    }
    return OD_OK;
}

static uint32_t rd_version(const od_entry_t *, uint32_t off, uint8_t *buf, size_t cap, size_t *len, uint32_t *total)
{
    return read_text(esp_app_get_description()->version, off, buf, cap, len, total);
}

static uint32_t rd_desc(const od_entry_t *, uint32_t off, uint8_t *buf, size_t cap, size_t *len, uint32_t *total)
{
    *total = (uint32_t)s_desc_len;
    if (s_desc == NULL || off > s_desc_len) {
        return OD_ABORT_LENGTH;
    }
    *len = s_desc_len - off < cap ? s_desc_len - off : cap;
    memcpy(buf, s_desc + off, *len);
    return OD_OK;
}

static uint32_t u32_of(const uint8_t *d)
{
    return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}

static uint32_t wr_save(const od_entry_t *, const uint8_t *d, size_t)
{
    if (u32_of(d) != SAVE_SIGNATURE) {
        return OD_ABORT_RANGE;
    }
    return robot_params_save() == ESP_OK ? OD_OK : OD_ABORT_STORE;
}

static uint32_t wr_load(const od_entry_t *, const uint8_t *d, size_t)
{
    if (u32_of(d) != LOAD_SIGNATURE) {
        return OD_ABORT_RANGE;
    }
    robot_params_reset();
    robot_params_apply();
    return OD_OK;
}

static size_t max_pdo_payload(void)
{
    return ble_remote_max_payload();
}

static uint32_t wr_tpdo_count(const od_entry_t *e, const uint8_t *d, size_t)
{
    od_tpdo_t *t = &g_tpdo[e->index & 3];
    return od_pdo_validate(&g_od, t->map, d[0], max_pdo_payload());
}

static uint32_t wr_tpdo_map(const od_entry_t *e, const uint8_t *, size_t)
{
    // As in CANopen: set the count to 0, change the words, then set the count again
    return g_tpdo[e->index & 3].count == 0 ? OD_OK : OD_ABORT_STATE;
}

static uint32_t wr_tpdo_enable(const od_entry_t *e, const uint8_t *d, size_t)
{
    od_tpdo_t *t = &g_tpdo[e->index & 3];
    if (d[0] == 0) {
        return OD_OK;
    }
    if (t->count == 0) {
        return OD_ABORT_STATE;
    }
    t->last_us = 0;
    return od_pdo_validate(&g_od, t->map, t->count, max_pdo_payload());
}

static uint32_t wr_tilt_limits(const od_entry_t *, const uint8_t *, size_t)
{
    return g_params.tilt_min_deg <= g_params.tilt_max_deg ? OD_OK : OD_ABORT_RANGE;
}

static uint32_t wr_pid(const od_entry_t *e, const uint8_t *, size_t)
{
    uint8_t id = (uint8_t)(e->index - 0x3100);
    if (g_params.pid[id - 1][6] > g_params.pid[id - 1][5]) {
        return OD_ABORT_RANGE;  // Umin above Umax
    }
    return app_motor_set_pid(id, g_params.pid[id - 1]) ? OD_OK : OD_ABORT_GENERAL;
}

static uint32_t wr_servo_origin(const od_entry_t *, const uint8_t *, size_t)
{
    set_servo_origins(g_params.servo_origin[0], g_params.servo_origin[1]);
    return OD_OK;
}

static uint32_t wr_debug_mode(const od_entry_t *, const uint8_t *d, size_t)
{
    return motor_debug_set_mode(d[0] != 0);
}

static uint32_t wr_debug_motor(const od_entry_t *, const uint8_t *d, size_t)
{
    return motor_debug_select(d[0]);
}

static uint32_t wr_debug_run(const od_entry_t *, const uint8_t *d, size_t)
{
    return motor_debug_set_run(d[0] != 0);
}

static uint32_t wr_estop(const od_entry_t *, const uint8_t *d, size_t)
{
    if (d[0]) {
        remote_link_emergency_stop();
    }
    return OD_OK;
}

static uint32_t wr_clear_estop(const od_entry_t *, const uint8_t *d, size_t)
{
    if (d[0]) {
        cmd_hub_set_estop(false);
    }
    return OD_OK;
}

static uint32_t gate(const od_entry_t *e, void *)
{
    if (!(e->flags & (OD_F_TAKEOVER | OD_F_IDLE))) {
        return OD_OK;
    }
    if (!remote_link_hw_ready()) {
        return OD_ABORT_STATE;  // BLE is up before the motor drivers
    }
    cmd_hub_status_t hub;
    cmd_hub_get_status(&hub);
    if ((e->flags & OD_F_TAKEOVER) && (hub.estop || !hub.takeover || !hub.fresh[CMD_SRC_BLE])) {
        return OD_ABORT_STATE;
    }
    if (e->flags & OD_F_IDLE) {
        if (hub.host_state != HOST_IDLE) {
            return OD_ABORT_STATE;
        }
        for (uint8_t id = APP_MOTOR_ID_FIRST; id <= APP_MOTOR_ID_LAST; id++) {
            app_motor_info_t info;
            if (app_motor_get_info(id, &info) && info.enabled) {
                return OD_ABORT_STATE;
            }
        }
    }
    return OD_OK;
}

// ---- table -----------------------------------------------------------------

#define PDO OD_F_PDO
#define STORE OD_F_STORE
#define IDLE OD_F_IDLE
#define CONFIRM OD_F_CONFIRM
#define RANGE OD_F_RANGE

// Read-only value backed by a variable
#define RO(i, s, t, f, n, u, p) {i, s, t, OD_R, (uint8_t)(f), n, u, 0, 0, NULL, (void *)(p), NULL, NULL}
#define RO_ENUM(i, s, f, n, en, p) {i, s, OD_U8, OD_R, (uint8_t)(f), n, NULL, 0, 0, en, (void *)(p), NULL, NULL}
// Read-write value with a range
#define RW(i, s, t, f, n, u, lo, hi, p, w) {i, s, t, OD_RW, (uint8_t)((f) | RANGE), n, u, lo, hi, NULL, (void *)(p), NULL, w}

#define TPDO_COMM(n) \
    RW(0x1800 + n, 1, OD_U16, 0, "周期", "ms", 10, 5000, &g_tpdo[n].period_ms, NULL), \
    {0x1800 + n, 2, OD_BOOL, OD_RW, 0, "启用", NULL, 0, 0, NULL, &g_tpdo[n].enabled, NULL, wr_tpdo_enable}

#define TPDO_MAP_WORD(n, k) \
    {0x1A00 + n, k, OD_U32, OD_RW, 0, "映射 " #k, NULL, 0, 0, NULL, &g_tpdo[n].map[k - 1], NULL, wr_tpdo_map}

#define TPDO_MAP(n) \
    RW(0x1A00 + n, 0, OD_U8, 0, "映射条数", NULL, 0, OD_PDO_MAX_MAP, &g_tpdo[n].count, wr_tpdo_count), \
    TPDO_MAP_WORD(n, 1), TPDO_MAP_WORD(n, 2), TPDO_MAP_WORD(n, 3), TPDO_MAP_WORD(n, 4), \
    TPDO_MAP_WORD(n, 5), TPDO_MAP_WORD(n, 6), TPDO_MAP_WORD(n, 7), TPDO_MAP_WORD(n, 8)

#define MOTOR(id) \
    RO(0x2200 + id, 1, OD_I16, PDO, "实测转速", "rpm", &live.motor[id - 1].speed), \
    RO(0x2200 + id, 2, OD_F32, PDO, "目标值", NULL, &live.motor[id - 1].target), \
    RO(0x2200 + id, 3, OD_F32, PDO, "电流", "A", &live.motor[id - 1].current), \
    RO(0x2200 + id, 4, OD_F32, PDO, "温度", "°C", &live.motor[id - 1].temperature), \
    RO(0x2200 + id, 5, OD_F32, PDO, "转子角度", "°", &live.motor[id - 1].angle), \
    RO(0x2200 + id, 6, OD_I8, PDO, "状态字", NULL, &live.motor[id - 1].status), \
    RO(0x2200 + id, 7, OD_BOOL, PDO, "使能", NULL, &live.motor[id - 1].enabled), \
    RO(0x2200 + id, 8, OD_U8, 0, "CAN ID", NULL, &live.motor[id - 1].can_id)

#define PID(id) \
    RW(0x3100 + id, 1, OD_F32, STORE | IDLE | CONFIRM, "Kp", NULL, 0, 10000, &g_params.pid[id - 1][0], wr_pid), \
    RW(0x3100 + id, 2, OD_F32, STORE | IDLE | CONFIRM, "Ki", NULL, 0, 10000, &g_params.pid[id - 1][1], wr_pid), \
    RW(0x3100 + id, 3, OD_F32, STORE | IDLE | CONFIRM, "Kd", NULL, 0, 10000, &g_params.pid[id - 1][2], wr_pid), \
    RW(0x3100 + id, 4, OD_F32, STORE | IDLE | CONFIRM, "Kr", NULL, 0, 10, &g_params.pid[id - 1][3], wr_pid), \
    RW(0x3100 + id, 5, OD_F32, STORE | IDLE | CONFIRM, "Km", NULL, 0, 10, &g_params.pid[id - 1][4], wr_pid), \
    RW(0x3100 + id, 6, OD_F32, STORE | IDLE | CONFIRM, "Umax", NULL, 0, 16384, &g_params.pid[id - 1][5], wr_pid), \
    RW(0x3100 + id, 7, OD_F32, STORE | IDLE | CONFIRM, "Umin", NULL, -16384, 0, &g_params.pid[id - 1][6], wr_pid)

// esp_reset_reason_t
static const char *const RESET_REASONS = "未知,上电,外部复位,软件重启,程序崩溃,中断看门狗,任务看门狗,其他看门狗,深睡唤醒,电源欠压,SDIO,USB,JTAG,eFuse,电源毛刺,CPU 锁死";
static const char *const STAGES = "启动中,启动蓝牙,初始化 IMU,启动 CAN 电机,初始化舵机,初始化步进电机,步进回零,启动串口,就绪";
static const char *const MCU_STATES = "空闲,找球中,已取球,射球中,射球完成,错误";
static const char *const HOST_STATES = "空闲,找球,到达球,射球,扫描,错误";
static const char *const SOURCES = "无,串口上位机,蓝牙遥控";
static const char *const DEBUG_MOTORS = "未选择,摩擦轮 1,摩擦轮 2,左轮,右轮";

static const od_entry_t entries[] = {
    {0x1008, 0, OD_STR, OD_R, 0, "设备名", NULL, 0, 0, NULL, NULL, rd_name, NULL},
    {0x100A, 0, OD_STR, OD_R, 0, "固件版本", NULL, 0, 0, NULL, NULL, rd_version, NULL},
    {0x1010, 1, OD_U32, OD_W, IDLE | CONFIRM, "保存全部参数", NULL, 0, 0, NULL, NULL, NULL, wr_save},
    {0x1011, 1, OD_U32, OD_W, IDLE | CONFIRM, "恢复默认参数", NULL, 0, 0, NULL, NULL, NULL, wr_load},
    {0x1021, 0, OD_DOMAIN, OD_R, 0, "字典描述", NULL, 0, 0, NULL, NULL, rd_desc, NULL},
    RO(0x1022, 0, OD_U32, 0, "字典校验和", NULL, &s_desc_crc),
    TPDO_COMM(0), TPDO_COMM(1), TPDO_COMM(2), TPDO_COMM(3),
    TPDO_MAP(0), TPDO_MAP(1), TPDO_MAP(2), TPDO_MAP(3),

    RO_ENUM(0x2000, 1, PDO, "MCU 状态", MCU_STATES, &live.mcu_state),
    RO_ENUM(0x2000, 2, PDO, "生效上位机状态", HOST_STATES, &live.host_state),
    RO_ENUM(0x2000, 3, PDO, "当前控制源", SOURCES, &live.source),
    RO(0x2000, 4, OD_BOOL, PDO, "急停锁存", NULL, &live.estop),
    RO(0x2000, 5, OD_BOOL, PDO, "蓝牙接管中", NULL, &live.takeover),
    RO(0x2000, 6, OD_U32, PDO, "运行时间", "ms", &live.uptime_ms),
    RO_ENUM(0x2001, 1, PDO, "开机阶段", STAGES, &live.stage),
    RO(0x2001, 2, OD_U32, PDO, "故障位", NULL, &live.faults),
    {0x2001, 3, OD_STR, OD_R, 0, "故障原因", NULL, 0, 0, NULL, NULL, rd_reason, NULL},
    RO_ENUM(0x2001, 4, 0, "上次复位原因", RESET_REASONS, &s_reset_reason),
    {0x2001, 5, OD_STR, OD_R, 0, "上次崩溃信息", NULL, 0, 0, NULL, NULL, rd_crash, NULL},
    {0x2001, 6, OD_BOOL, OD_W, 0, "清除崩溃记录", NULL, 0, 0, NULL, NULL, NULL, wr_clear_crash},

    MOTOR(1), MOTOR(2), MOTOR(3), MOTOR(4), MOTOR(5),
    RO(0x2205, 9, OD_F32, PDO, "拨球轴角度", "°", &live.loader_shaft),
    RO(0x2205, 10, OD_F32, PDO, "角度环误差", "°", &live.loader_error),

    RO(0x2300, 1, OD_F32, PDO, "仰角（指令）", "°", &live.tilt),
    RO(0x2300, 2, OD_BOOL, PDO, "舵机通电", NULL, &live.servos_on),

    RO(0x2400, 1, OD_I32, PDO, "当前位置", "step", &live.stepper_pos),
    RO(0x2400, 2, OD_I32, PDO, "目标位置", "step", &live.stepper_target),
    RO(0x2400, 3, OD_BOOL, PDO, "限位开关", NULL, &live.endstop),
    RO(0x2400, 4, OD_BOOL, PDO, "运动中", NULL, &live.stepper_running),
    RO(0x2400, 5, OD_BOOL, PDO, "已回零", NULL, &live.stepper_homed),

    RO(0x2500, 1, OD_I16, PDO, "加速度 X", "raw", &live.acc[0]),
    RO(0x2500, 2, OD_I16, PDO, "加速度 Y", "raw", &live.acc[1]),
    RO(0x2500, 3, OD_I16, PDO, "加速度 Z", "raw", &live.acc[2]),
    RO(0x2500, 4, OD_I16, PDO, "角速度 X", "raw", &live.gyr[0]),
    RO(0x2500, 5, OD_I16, PDO, "角速度 Y", "raw", &live.gyr[1]),
    RO(0x2500, 6, OD_I16, PDO, "角速度 Z", "raw", &live.gyr[2]),
    RO(0x2500, 7, OD_U32, PDO, "采样序号", NULL, &live.imu_seq),

    {0x2600, 1, OD_BOOL, OD_RW, OD_F_SELF, "调试模式", NULL, 0, 0, NULL, &g_motor_debug.mode, NULL, wr_debug_mode},
    {0x2600, 2, OD_U8, OD_RW, RANGE | OD_F_SELF, "调试电机", NULL, 0, 4, DEBUG_MOTORS, &g_motor_debug.motor_id, NULL, wr_debug_motor},
    RW(0x2600, 3, OD_F32, PDO, "目标转速", "rpm", -9000, 9000, &g_motor_debug.target_rpm, NULL),
    {0x2600, 4, OD_BOOL, OD_RW, PDO | OD_F_SELF, "运行（按住）", NULL, 0, 0, NULL, &g_motor_debug.run, NULL, wr_debug_run},
    RW(0x2600, 5, OD_U16, STORE, "调试转速上限", "rpm", 0, 9000, &g_params.debug_max_rpm, NULL),
    RO(0x2600, 6, OD_F32, PDO, "实际下发转速", "rpm", &g_motor_debug.applied_rpm),
    RO(0x2600, 7, OD_I16, PDO, "实测转速", "rpm", &live.debug_speed),
    RO(0x2600, 8, OD_F32, PDO, "电流", "A", &live.debug_current),
    RO(0x2600, 9, OD_F32, PDO, "温度", "°C", &live.debug_temperature),

    RW(0x3000, 1, OD_U16, STORE, "遥控轮速上限", "rpm", 0, 9000, &g_params.max_wheel_rpm, NULL),
    RW(0x3000, 2, OD_U16, STORE, "遥控摩擦轮上限", NULL, 0, 9000, &g_params.max_shoot_speed, NULL),
    RW(0x3000, 3, OD_I16, STORE, "遥控仰角下限", "°", -10, 40, &g_params.tilt_min_deg, wr_tilt_limits),
    RW(0x3000, 4, OD_I16, STORE, "遥控仰角上限", "°", -10, 40, &g_params.tilt_max_deg, wr_tilt_limits),

    PID(1), PID(2), PID(3), PID(4), PID(5), PID(6),

    RW(0x3200, 1, OD_F32, STORE | IDLE, "舵机 1 中位", "°", 60, 120, &g_params.servo_origin[0], wr_servo_origin),
    RW(0x3200, 2, OD_F32, STORE | IDLE, "舵机 2 中位", "°", 60, 120, &g_params.servo_origin[1], wr_servo_origin),

    RW(0x3201, 1, OD_I16, STORE, "摩擦轮速度", NULL, -9000, 9000, &g_params.search_friction_speed, NULL),
    RW(0x3201, 2, OD_I32, STORE, "步进位置", "step", 0, 5000, &g_params.search_stepper_pos, NULL),
    RW(0x3201, 3, OD_F32, STORE, "仰角（压下）", "°", -10, 40, &g_params.search_tilt_down_deg, NULL),
    RW(0x3201, 4, OD_F32, STORE, "仰角（保持）", "°", -10, 40, &g_params.search_tilt_hold_deg, NULL),
    RW(0x3201, 5, OD_U16, STORE, "压下等待", "ms", 0, 5000, &g_params.search_tilt_ms, NULL),

    RW(0x3202, 1, OD_I16, STORE, "摩擦轮速度", NULL, -9000, 9000, &g_params.pickup_friction_speed, NULL),
    RW(0x3202, 2, OD_F32, STORE, "仰角", "°", -10, 40, &g_params.pickup_tilt_deg, NULL),
    RW(0x3202, 3, OD_U16, STORE, "仰角等待", "ms", 0, 5000, &g_params.pickup_tilt_ms, NULL),
    RW(0x3202, 4, OD_F32, STORE, "拨球角度 1", "°", -180, 180, &g_params.pickup_loader1_deg, NULL),
    RW(0x3202, 5, OD_U16, STORE, "拨球等待 1", "ms", 0, 5000, &g_params.pickup_loader1_ms, NULL),
    RW(0x3202, 6, OD_F32, STORE, "拨球角度 2", "°", -180, 180, &g_params.pickup_loader2_deg, NULL),
    RW(0x3202, 7, OD_U16, STORE, "拨球等待 2", "ms", 0, 5000, &g_params.pickup_loader2_ms, NULL),

    RW(0x3203, 1, OD_I32, STORE, "步进位置", "step", 0, 5000, &g_params.shoot_stepper_pos, NULL),
    RW(0x3203, 2, OD_U16, STORE, "摩擦轮加速等待", "ms", 0, 5000, &g_params.shoot_spinup_ms, NULL),
    RW(0x3203, 3, OD_F32, STORE, "拨球角度 1", "°", -180, 180, &g_params.shoot_loader1_deg, NULL),
    RW(0x3203, 4, OD_U16, STORE, "拨球等待 1", "ms", 0, 5000, &g_params.shoot_loader1_ms, NULL),
    RW(0x3203, 5, OD_F32, STORE, "拨球角度 2", "°", -180, 180, &g_params.shoot_loader2_deg, NULL),
    RW(0x3203, 6, OD_U16, STORE, "拨球等待 2", "ms", 0, 5000, &g_params.shoot_loader2_ms, NULL),

    {0x5000, 1, OD_BOOL, OD_W, 0, "急停", NULL, 0, 0, NULL, NULL, NULL, wr_estop},
    {0x5000, 2, OD_BOOL, OD_W, CONFIRM, "解除急停", NULL, 0, 0, NULL, NULL, NULL, wr_clear_estop},
};

static const od_object_t objects[] = {
    {0x1008, "设备名"},
    {0x100A, "固件版本"},
    {0x1010, "保存参数"},
    {0x1011, "恢复默认参数"},
    {0x1021, "字典描述"},
    {0x1022, "字典校验和"},
    {0x1800, "TPDO1 通信参数"},
    {0x1801, "TPDO2 通信参数"},
    {0x1802, "TPDO3 通信参数"},
    {0x1803, "TPDO4 通信参数"},
    {0x1A00, "TPDO1 映射"},
    {0x1A01, "TPDO2 映射"},
    {0x1A02, "TPDO3 映射"},
    {0x1A03, "TPDO4 映射"},
    {0x2000, "机器人状态"},
    {0x2001, "诊断"},
    {0x2201, "摩擦轮 1（CAN 1）"},
    {0x2202, "摩擦轮 2（CAN 2）"},
    {0x2203, "左轮（CAN 3）"},
    {0x2204, "右轮（CAN 4）"},
    {0x2205, "拨球（CAN 5）"},
    {0x2300, "舵机"},
    {0x2400, "步进电机"},
    {0x2500, "IMU"},
    {0x2600, "电机调试"},
    {0x3000, "遥控限幅"},
    {0x3101, "PID 摩擦轮 1"},
    {0x3102, "PID 摩擦轮 2"},
    {0x3103, "PID 左轮"},
    {0x3104, "PID 右轮"},
    {0x3105, "PID 拨球角度环"},
    {0x3106, "PID 拨球速度环"},
    {0x3200, "舵机中位"},
    {0x3201, "找球动作"},
    {0x3202, "取球动作"},
    {0x3203, "射球动作"},
    {0x5000, "急停"},
};

const od_dict_t g_od = {
    entries, sizeof(entries) / sizeof(entries[0]),
    objects, sizeof(objects) / sizeof(objects[0]),
    gate, NULL,
};

void od_table_reset_tpdo(void)
{
    memset(g_tpdo, 0, sizeof(g_tpdo));
    // TPDO1: robot status and both drive wheels, like the fixed Status frame
    g_tpdo[0].period_ms = 100;
    g_tpdo[0].count = 8;
    const uint32_t status_map[] = {
        od_map(0x2000, 1, 8), od_map(0x2000, 2, 8), od_map(0x2000, 3, 8), od_map(0x2000, 4, 8),
        od_map(0x2000, 5, 8), od_map(0x2000, 6, 32), od_map(0x2203, 1, 16), od_map(0x2204, 1, 16),
    };
    memcpy(g_tpdo[0].map, status_map, sizeof(status_map));
    // TPDO2: the motor being debugged
    g_tpdo[1].period_ms = 20;
    g_tpdo[1].count = 5;
    const uint32_t debug_map[] = {
        od_map(0x2600, 6, 32), od_map(0x2600, 7, 16), od_map(0x2600, 8, 32), od_map(0x2600, 9, 32),
        od_map(0x2600, 4, 8),
    };
    memcpy(g_tpdo[1].map, debug_map, sizeof(debug_map));
    // TPDO3: IMU
    g_tpdo[2].period_ms = 20;
    g_tpdo[2].count = 7;
    for (uint8_t i = 0; i < 6; i++) {
        g_tpdo[2].map[i] = od_map(0x2500, (uint8_t)(i + 1), 16);
    }
    g_tpdo[2].map[6] = od_map(0x2500, 7, 32);
    // TPDO4: free for the app
    g_tpdo[3].period_ms = 100;
}

void od_table_init(void)
{
    for (size_t i = 1; i < g_od.count; i++) {
        const od_entry_t *a = &entries[i - 1], *b = &entries[i];
        if (((uint32_t)a->index << 8 | a->sub) >= ((uint32_t)b->index << 8 | b->sub)) {
            // Lookups would miss entries; complain loudly but keep the robot running
            ESP_LOGE(TAG, "Table not sorted at 0x%04X.%u, fix od_table.cpp", b->index, b->sub);
        }
    }
    for (uint8_t id = APP_MOTOR_ID_FIRST; id <= APP_MOTOR_ID_LAST; id++) {
        live.motor[id - 1].can_id = id;
    }
    od_table_reset_tpdo();

    s_desc_len = od_describe(&g_od, NULL, 0);
    s_desc = (char *)malloc(s_desc_len + 1);
    if (s_desc == NULL) {
        ESP_LOGE(TAG, "No memory for the descriptor (%u bytes)", (unsigned)s_desc_len);
        s_desc_len = 0;
        return;
    }
    od_describe(&g_od, s_desc, s_desc_len + 1);
    s_desc_crc = od_crc32((const uint8_t *)s_desc, s_desc_len);
    ESP_LOGI(TAG, "%u entries, descriptor %u bytes, crc %08lX", (unsigned)g_od.count, (unsigned)s_desc_len,
             (unsigned long)s_desc_crc);
}

void od_table_refresh(void)
{
    cmd_hub_status_t hub;
    cmd_hub_get_status(&hub);
    live.mcu_state = (uint8_t)*getMcuState();
    live.host_state = hub.host_state;
    if (hub.takeover && hub.fresh[CMD_SRC_BLE]) {
        live.source = 2;
    } else if (!hub.takeover && hub.fresh[CMD_SRC_SERIAL]) {
        live.source = 1;
    } else {
        live.source = 0;
    }
    live.estop = hub.estop;
    live.takeover = hub.takeover;
    live.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    live.stage = (uint8_t)health_stage();
    s_reset_reason = health_reset_reason();
    live.faults = health_faults();

    for (uint8_t id = APP_MOTOR_ID_FIRST; id <= APP_MOTOR_ID_LAST; id++) {
        app_motor_info_t info;
        if (app_motor_get_info(id, &info)) {
            motor_live_t *m = &live.motor[id - 1];
            m->speed = info.speed_rpm;
            m->target = info.target;
            m->current = info.current_a;
            m->temperature = info.temperature_c;
            m->angle = info.angle_deg;
            m->status = info.status;
            m->enabled = info.enabled;
        }
    }
    live.loader_shaft = app_loader_shaft_angle();
    live.loader_error = app_loader_pid_error();
    live.tilt = get_tilt_angle();
    live.servos_on = servos_enabled();
    live.stepper_pos = get_stepper_pos();
    live.stepper_target = get_stepper_target();
    live.endstop = stepper_endstop_hit();
    live.stepper_running = stepper_is_running();
    live.stepper_homed = stepper_is_homed();

    t_sQMI8658 imu;
    get_latest_imu_sample(&imu, &live.imu_seq);
    live.acc[0] = imu.acc_x;
    live.acc[1] = imu.acc_y;
    live.acc[2] = imu.acc_z;
    live.gyr[0] = imu.gyr_x;
    live.gyr[1] = imu.gyr_y;
    live.gyr[2] = imu.gyr_z;

    uint8_t dbg = g_motor_debug.motor_id;
    if (dbg >= APP_MOTOR_ID_FIRST && dbg <= APP_MOTOR_ID_LAST) {
        live.debug_speed = live.motor[dbg - 1].speed;
        live.debug_current = live.motor[dbg - 1].current;
        live.debug_temperature = live.motor[dbg - 1].temperature;
    } else {
        live.debug_speed = 0;
        live.debug_current = 0;
        live.debug_temperature = 0;
    }
}

#endif // CONFIG_HELLOBALLS_BLE_REMOTE
