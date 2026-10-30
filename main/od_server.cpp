#include "od_server.h"

#include "sdkconfig.h"

#if CONFIG_HELLOBALLS_BLE_REMOTE

#include <string.h>
#include "ble_remote.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "motor_debug.h"
#include "obj_dict.h"
#include "od_table.h"
#include "health.h"
#include "remote_link.h"

#define TAG "OD_SERVER"
#define TICK_MS 10
#define QUEUE_LEN 8
#define MAX_REQUEST 64  // an SDO request carries at most a short value or an offset

typedef struct {
    uint8_t len;
    uint8_t data[MAX_REQUEST];
} sdo_request_t;

static QueueHandle_t s_queue;
static volatile bool s_reset;
static uint8_t s_health_ticks;
#define HEALTH_TICKS 10  // faults re-evaluated every 100 ms  // a phone connected or left: start from a clean state

void od_server_on_sdo(const uint8_t *data, size_t len)
{
    if (s_queue == NULL || len > MAX_REQUEST) {
        return;  // the app times out and retries
    }
    sdo_request_t req;
    req.len = (uint8_t)len;
    memcpy(req.data, data, len);
    if (xQueueSend(s_queue, &req, 0) != pdTRUE) {
        ESP_LOGW(TAG, "SDO queue full, request dropped");
    }
}

void od_server_on_connection(bool connected)
{
    (void)connected;
    s_reset = true;
}

static void send_tpdos(int64_t now)
{
    bool refreshed = false;
    for (uint8_t n = 0; n < OD_TPDO_COUNT; n++) {
        od_tpdo_t *t = &g_tpdo[n];
        if (!t->enabled || t->count == 0) {
            continue;
        }
        if (t->last_us != 0 && now - t->last_us < (int64_t)t->period_ms * 1000) {
            continue;
        }
        t->last_us = now;
        if (!refreshed) {
            od_table_refresh();
            refreshed = true;
        }
        uint8_t buf[BLE_REMOTE_MAX_PAYLOAD];
        size_t len = od_pdo_pack(&g_od, (uint8_t)(n + 1), t->seq++, (uint16_t)(now / 1000), t->map, t->count,
                                 buf, ble_remote_max_payload());
        if (len > 0) {
            ble_remote_notify_tpdo(buf, len);
        }
    }
}

static void od_task(void *arg)
{
    int64_t next_tick = esp_timer_get_time();
    while (true) {
        int64_t now = esp_timer_get_time();
        TickType_t wait = 0;
        if (next_tick > now) {
            wait = pdMS_TO_TICKS((next_tick - now) / 1000);
        }

        sdo_request_t req;
        if (xQueueReceive(s_queue, &req, wait) == pdTRUE) {
            od_table_refresh();
            uint8_t rsp[BLE_REMOTE_MAX_PAYLOAD];
            size_t len = od_sdo_handle(&g_od, req.data, req.len, rsp, ble_remote_max_payload());
            if (len > 0) {
                ble_remote_notify_sdo(rsp, len);
            }
        }

        now = esp_timer_get_time();
        if (now < next_tick) {
            continue;
        }
        next_tick = now + TICK_MS * 1000;

        if (s_reset) {
            s_reset = false;
            motor_debug_exit("connection changed");
            od_table_reset_tpdo();
        }
        if (remote_link_hw_ready()) {
            motor_debug_tick(TICK_MS);
        }
        if (++s_health_ticks >= HEALTH_TICKS) {
            s_health_ticks = 0;
            health_refresh();
        }
        send_tpdos(now);
    }
}

void od_server_init(void)
{
    od_table_init();
    s_queue = xQueueCreate(QUEUE_LEN, sizeof(sdo_request_t));
    xTaskCreate(od_task, "od_server", 6144, NULL, 4, NULL);
}

#endif // CONFIG_HELLOBALLS_BLE_REMOTE
