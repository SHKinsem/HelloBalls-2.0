#include "ble_remote_codec.h"

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)(v >> 24);
}

static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool ble_remote_decode_control(const uint8_t *buf, size_t len, ble_remote_cmd_t *out)
{
    if (buf == NULL || out == NULL || len != BLE_REMOTE_CONTROL_LEN || buf[0] != BLE_REMOTE_PROTOCOL_VERSION) {
        return false;
    }
    out->seq = buf[1];
    out->state = buf[2];
    out->flags = buf[3];
    out->wheel1_rpm = get_i16(&buf[4]);
    out->wheel2_rpm = get_i16(&buf[6]);
    out->tilt_angle = get_i16(&buf[8]);
    out->shoot_speed = get_i16(&buf[10]);
    return true;
}

size_t ble_remote_encode_control(const ble_remote_cmd_t *cmd, uint8_t *buf, size_t cap)
{
    if (cmd == NULL || buf == NULL || cap < BLE_REMOTE_CONTROL_LEN) {
        return 0;
    }
    buf[0] = BLE_REMOTE_PROTOCOL_VERSION;
    buf[1] = cmd->seq;
    buf[2] = cmd->state;
    buf[3] = cmd->flags;
    put_u16(&buf[4], (uint16_t)cmd->wheel1_rpm);
    put_u16(&buf[6], (uint16_t)cmd->wheel2_rpm);
    put_u16(&buf[8], (uint16_t)cmd->tilt_angle);
    put_u16(&buf[10], (uint16_t)cmd->shoot_speed);
    return BLE_REMOTE_CONTROL_LEN;
}

bool ble_remote_decode_action(const uint8_t *buf, size_t len, ble_remote_action_t *out)
{
    if (buf == NULL || out == NULL || len != BLE_REMOTE_ACTION_LEN || buf[0] != BLE_REMOTE_PROTOCOL_VERSION) {
        return false;
    }
    out->action = buf[1];
    out->param = get_i16(&buf[2]);
    return true;
}

size_t ble_remote_encode_action(const ble_remote_action_t *act, uint8_t *buf, size_t cap)
{
    if (act == NULL || buf == NULL || cap < BLE_REMOTE_ACTION_LEN) {
        return 0;
    }
    buf[0] = BLE_REMOTE_PROTOCOL_VERSION;
    buf[1] = act->action;
    put_u16(&buf[2], (uint16_t)act->param);
    return BLE_REMOTE_ACTION_LEN;
}

bool ble_remote_decode_status(const uint8_t *buf, size_t len, ble_remote_status_t *out)
{
    if (buf == NULL || out == NULL || len != BLE_REMOTE_STATUS_LEN || buf[0] != BLE_REMOTE_PROTOCOL_VERSION) {
        return false;
    }
    out->mcu_state = buf[1];
    out->host_state = buf[2];
    out->active_src = buf[3];
    out->flags = buf[4];
    out->last_seq = buf[5];
    out->wheel1_rpm = get_i16(&buf[6]);
    out->wheel2_rpm = get_i16(&buf[8]);
    out->uptime_ms = get_u32(&buf[10]);
    return true;
}

size_t ble_remote_encode_status(const ble_remote_status_t *st, uint8_t *buf, size_t cap)
{
    if (st == NULL || buf == NULL || cap < BLE_REMOTE_STATUS_LEN) {
        return 0;
    }
    buf[0] = BLE_REMOTE_PROTOCOL_VERSION;
    buf[1] = st->mcu_state;
    buf[2] = st->host_state;
    buf[3] = st->active_src;
    buf[4] = st->flags;
    buf[5] = st->last_seq;
    put_u16(&buf[6], (uint16_t)st->wheel1_rpm);
    put_u16(&buf[8], (uint16_t)st->wheel2_rpm);
    put_u32(&buf[10], st->uptime_ms);
    return BLE_REMOTE_STATUS_LEN;
}
