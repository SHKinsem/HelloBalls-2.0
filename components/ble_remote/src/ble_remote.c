#include "ble_remote.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_REMOTE";

// b5f9000X-7a4c-4e57-9c1d-48a7a3b4c0de, bytes in little-endian order for NimBLE
#define HB_UUID(x) BLE_UUID128_INIT(0xde, 0xc0, 0xb4, 0xa3, 0xa7, 0x48, 0x1d, 0x9c, \
                                    0x57, 0x4e, 0x4c, 0x7a, (x), 0x00, 0xf9, 0xb5)

static const ble_uuid128_t svc_uuid = HB_UUID(0x01);
static const ble_uuid128_t control_uuid = HB_UUID(0x02);
static const ble_uuid128_t action_uuid = HB_UUID(0x03);
static const ble_uuid128_t status_uuid = HB_UUID(0x04);
static const ble_uuid128_t info_uuid = HB_UUID(0x05);
static const ble_uuid128_t sdo_uuid = HB_UUID(0x06);
static const ble_uuid128_t tpdo_uuid = HB_UUID(0x08);

static ble_remote_config_t s_cfg;
static char s_name[32];
static char s_info[64];
static uint8_t s_own_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_status_handle;
static bool s_status_subscribed;
static uint16_t s_sdo_handle;
static bool s_sdo_subscribed;
static uint16_t s_tpdo_handle;
static bool s_tpdo_subscribed;
static uint16_t s_mtu = 23;

static ble_remote_status_t s_status;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;

static void advertise(void);

static int read_flat(struct os_mbuf *om, uint8_t *buf, uint16_t cap, uint16_t *len)
{
    uint16_t om_len = OS_MBUF_PKTLEN(om);
    if (om_len > cap) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (ble_hs_mbuf_to_flat(om, buf, cap, len) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    uint8_t buf[BLE_REMOTE_STATUS_LEN];
    uint16_t len = 0;

    if (ble_uuid_cmp(uuid, &control_uuid.u) == 0 && ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        int rc = read_flat(ctxt->om, buf, sizeof(buf), &len);
        if (rc != 0) {
            return rc;
        }
        ble_remote_cmd_t cmd;
        if (!ble_remote_decode_control(buf, len, &cmd)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (s_cfg.on_command) {
            s_cfg.on_command(&cmd, s_cfg.ctx);
        }
        return 0;
    }

    if (ble_uuid_cmp(uuid, &action_uuid.u) == 0 && ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        int rc = read_flat(ctxt->om, buf, sizeof(buf), &len);
        if (rc != 0) {
            return rc;
        }
        ble_remote_action_t action;
        if (!ble_remote_decode_action(buf, len, &action)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        ESP_LOGI(TAG, "Action 0x%02x param %d", action.action, action.param);
        if (s_cfg.on_action) {
            s_cfg.on_action(&action, s_cfg.ctx);
        }
        return 0;
    }

    if (ble_uuid_cmp(uuid, &sdo_uuid.u) == 0 && ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t req[BLE_REMOTE_MAX_PAYLOAD];
        int rc = read_flat(ctxt->om, req, sizeof(req), &len);
        if (rc != 0) {
            return rc;
        }
        if (s_cfg.on_sdo) {
            s_cfg.on_sdo(req, len, s_cfg.ctx);
        }
        return 0;
    }

    if (ble_uuid_cmp(uuid, &status_uuid.u) == 0 && ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        ble_remote_status_t st;
        taskENTER_CRITICAL(&s_status_lock);
        st = s_status;
        taskEXIT_CRITICAL(&s_status_lock);
        len = (uint16_t)ble_remote_encode_status(&st, buf, sizeof(buf));
        return os_mbuf_append(ctxt->om, buf, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ble_uuid_cmp(uuid, &info_uuid.u) == 0 && ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, s_info, strlen(s_info)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &control_uuid.u,
                .access_cb = gatt_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &action_uuid.u,
                .access_cb = gatt_access,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &status_uuid.u,
                .access_cb = gatt_access,
                .val_handle = &s_status_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &info_uuid.u,
                .access_cb = gatt_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &sdo_uuid.u,
                .access_cb = gatt_access,
                .val_handle = &s_sdo_handle,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &tpdo_uuid.u,
                .access_cb = gatt_access,
                .val_handle = &s_tpdo_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static void set_connected(uint16_t handle)
{
    bool connected = handle != BLE_HS_CONN_HANDLE_NONE;
    s_conn_handle = handle;
    s_status_subscribed = false;
    s_sdo_subscribed = false;
    s_tpdo_subscribed = false;
    s_mtu = 23;
    if (s_cfg.on_connection) {
        s_cfg.on_connection(connected, s_cfg.ctx);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connected, handle %d", event->connect.conn_handle);
            set_connected(event->connect.conn_handle);
        } else {
            advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, reason 0x%x", event->disconnect.reason);
        set_connected(BLE_HS_CONN_HANDLE_NONE);
        advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_status_handle) {
            s_status_subscribed = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_sdo_handle) {
            s_sdo_subscribed = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_tpdo_handle) {
            s_tpdo_subscribed = event->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            advertise();
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU %d", event->mtu.value);
        s_mtu = event->mtu.value;
        return 0;

    default:
        return 0;
    }
}

static void advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)s_name;
    fields.name_len = strlen(s_name);
    fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv fields rc=%d", rc);
        return;
    }

    // The 128-bit service UUID does not fit next to the name, so it goes in the scan response
    struct ble_hs_adv_fields rsp = {0};
    rsp.uuids128 = (ble_uuid128_t *)&svc_uuid;
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "scan rsp fields rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv start rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Advertising as %s", s_name);
}

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        ESP_LOGE(TAG, "Failed to infer address type");
        return;
    }
    advertise();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset, reason %d", reason);
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_remote_init(const ble_remote_config_t *cfg)
{
    if (cfg != NULL) {
        s_cfg = *cfg;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BT);
    const char *prefix = s_cfg.name_prefix ? s_cfg.name_prefix : CONFIG_BLE_REMOTE_NAME_PREFIX;
    snprintf(s_name, sizeof(s_name), "%s-%02X%02X", prefix, mac[4], mac[5]);
    snprintf(s_info, sizeof(s_info), "proto=%d;od=1;fw=%s", BLE_REMOTE_PROTOCOL_VERSION,
             s_cfg.fw_version ? s_cfg.fw_version : "unknown");

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_att_set_preferred_mtu(BLE_REMOTE_PREFERRED_MTU);
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(gatt_svcs);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT setup failed, rc=%d", rc);
        return ESP_FAIL;
    }
    ble_svc_gap_device_name_set(s_name);

    nimble_port_freertos_init(host_task);
    return ESP_OK;
}

esp_err_t ble_remote_notify_status(const ble_remote_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    taskENTER_CRITICAL(&s_status_lock);
    s_status = *status;
    taskEXIT_CRITICAL(&s_status_lock);

    uint16_t conn = s_conn_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE || !s_status_subscribed) {
        return ESP_OK;
    }

    uint8_t buf[BLE_REMOTE_STATUS_LEN];
    size_t len = ble_remote_encode_status(status, buf, sizeof(buf));
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (om == NULL) {
        return ESP_ERR_NO_MEM;
    }
    return ble_gatts_notify_custom(conn, s_status_handle, om) == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t notify(uint16_t handle, bool subscribed, const uint8_t *data, size_t len)
{
    if (data == NULL || len > ble_remote_max_payload()) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t conn = s_conn_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE || !subscribed) {
        return ESP_ERR_INVALID_STATE;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        return ESP_ERR_NO_MEM;
    }
    return ble_gatts_notify_custom(conn, handle, om) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t ble_remote_notify_sdo(const uint8_t *data, size_t len)
{
    return notify(s_sdo_handle, s_sdo_subscribed, data, len);
}

esp_err_t ble_remote_notify_tpdo(const uint8_t *data, size_t len)
{
    return notify(s_tpdo_handle, s_tpdo_subscribed, data, len);
}

size_t ble_remote_max_payload(void)
{
    size_t n = s_mtu > 3 ? (size_t)s_mtu - 3 : 20;
    return n < BLE_REMOTE_MAX_PAYLOAD ? n : BLE_REMOTE_MAX_PAYLOAD;
}

bool ble_remote_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

const char *ble_remote_device_name(void)
{
    return s_name;
}
