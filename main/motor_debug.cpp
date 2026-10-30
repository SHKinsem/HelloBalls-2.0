#include "motor_debug.h"

#include "sdkconfig.h"

#if CONFIG_HELLOBALLS_BLE_REMOTE

#include "app_motors.h"
#include "cmd_hub.h"
#include "esp_log.h"
#include "obj_dict.h"
#include "robot_params.h"
#include "serial.h"

#define TAG "MOTOR_DEBUG"

motor_debug_t g_motor_debug = {};

bool motor_debug_active(void)
{
    return g_motor_debug.mode;
}

// Debug mode may run only while the BLE remote is in control and the robot is otherwise idle
static bool allowed(const char **why)
{
    cmd_hub_status_t hub;
    cmd_hub_get_status(&hub);
    if (hub.estop) {
        *why = "emergency stop";
        return false;
    }
    if (!hub.takeover || !hub.fresh[CMD_SRC_BLE]) {
        *why = "remote not in control";
        return false;
    }
    if (hub.host_state != HOST_IDLE) {
        *why = "host state not IDLE";
        return false;
    }
    return true;
}

static void stop_motors(void)
{
    g_motor_debug.applied_rpm = 0;
    app_motor_debug_drive(0, 0);
    motorStateHelper(false);
}

static void apply_servos(void)
{
    float t = g_motor_debug.servo_tilt;
    set_servo_offsets(t + g_motor_debug.servo_trim[0], -t + g_motor_debug.servo_trim[1]);
    g_motor_debug.servo_touched = true;
}

static void reset_servo_state(void)
{
    g_motor_debug.servo_tilt = 0;
    g_motor_debug.servo_trim[0] = 0;
    g_motor_debug.servo_trim[1] = 0;
}

uint32_t motor_debug_set_servo(uint8_t sub, float deg)
{
    if (!g_motor_debug.mode) {
        return OD_ABORT_STATE;
    }
    if (sub == 1) {
        g_motor_debug.servo_tilt = deg;
    } else if (sub == 2 || sub == 3) {
        g_motor_debug.servo_trim[sub - 2] = deg;
    } else {
        return OD_ABORT_NO_SUB;
    }
    apply_servos();
    return OD_OK;
}

void motor_debug_servo_origin_changed(void)
{
    if (g_motor_debug.mode && g_motor_debug.servo_touched) {
        apply_servos();
    } else {
        move_servos_to_origin();
    }
}

uint32_t motor_debug_set_mode(bool on)
{
    if (!on) {
        if (g_motor_debug.mode) {
            motor_debug_exit("requested");
        }
        return OD_OK;
    }
    const char *why = "";
    if (!allowed(&why)) {
        ESP_LOGW(TAG, "Refused: %s", why);
        return OD_ABORT_STATE;
    }
    g_motor_debug.mode = true;
    g_motor_debug.run = false;
    g_motor_debug.target_rpm = 0;
    reset_servo_state();
    g_motor_debug.servo_touched = false;
    stop_motors();
    ESP_LOGI(TAG, "Debug mode on");
    return OD_OK;
}

uint32_t motor_debug_select(uint8_t motor_id)
{
    if (motor_id == APP_MOTOR_ID_LOADER || motor_id > APP_MOTOR_ID_LAST) {
        return OD_ABORT_RANGE;
    }
    if (motor_id != g_motor_debug.motor_id) {
        // Never carry a running speed over to another motor
        g_motor_debug.run = false;
        g_motor_debug.target_rpm = 0;
        if (g_motor_debug.mode) {
            stop_motors();
        }
    }
    g_motor_debug.motor_id = motor_id;
    return OD_OK;
}

uint32_t motor_debug_set_run(bool run)
{
    if (run && (!g_motor_debug.mode || g_motor_debug.motor_id == 0)) {
        return OD_ABORT_STATE;
    }
    g_motor_debug.run = run;
    return OD_OK;
}

void motor_debug_exit(const char *why)
{
    if (!g_motor_debug.mode) {
        return;
    }
    g_motor_debug.mode = false;
    g_motor_debug.run = false;
    g_motor_debug.target_rpm = 0;
    stop_motors();
    if (g_motor_debug.servo_touched) {
        // Back to centre, then unpowered like after a command timeout
        reset_servo_state();
        move_servos_to_origin();
        disable_servos();
        g_motor_debug.servo_touched = false;
    }
    ESP_LOGI(TAG, "Debug mode off: %s", why);
}

void motor_debug_tick(uint32_t dt_ms)
{
    if (!g_motor_debug.mode) {
        return;
    }
    const char *why = "";
    if (!allowed(&why)) {
        motor_debug_exit(why);
        return;
    }

    float goal = 0;
    if (g_motor_debug.run && g_motor_debug.motor_id != 0) {
        float cap = g_params.debug_max_rpm;
        goal = g_motor_debug.target_rpm;
        goal = goal > cap ? cap : (goal < -cap ? -cap : goal);
    }
    float step = MOTOR_DEBUG_RAMP_RPM_PER_S * dt_ms / 1000.0f;
    float cur = g_motor_debug.applied_rpm;
    if (goal == 0 && !g_motor_debug.run) {
        cur = 0;  // releasing the button stops at once
    } else if (goal > cur + step) {
        cur += step;
    } else if (goal < cur - step) {
        cur -= step;
    } else {
        cur = goal;
    }
    g_motor_debug.applied_rpm = cur;

    if (g_motor_debug.run && g_motor_debug.motor_id != 0) {
        app_motor_debug_drive(g_motor_debug.motor_id, cur);
    } else {
        app_motor_debug_drive(0, 0);
    }
}

#else

bool motor_debug_active(void) { return false; }

#endif // CONFIG_HELLOBALLS_BLE_REMOTE
