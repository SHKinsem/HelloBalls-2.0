/* Host tests for cmd_hub arbitration, timeouts and emergency stop.
 * Build and run: components/cmd_hub/test/run_tests.sh */
#include <stdio.h>
#include <stdlib.h>
#include "cmd_hub.h"
#include "freertos/semphr.h"

static int64_t now_us = 1000000;
int64_t esp_timer_get_time(void) { return now_us; }

static struct { int count; } sems[4];
static int sem_used;
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return (SemaphoreHandle_t)&sems[sem_used++]; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { s->count = 1; return pdTRUE; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) {
    (void)t;
    if (s->count) { s->count = 0; return pdTRUE; }
    return pdFALSE;
}

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static robot_cmd_t mk(uint8_t state, int16_t w) { robot_cmd_t c = {state, w, w, 0, 0}; return c; }
static void advance_ms(int ms) { now_us += (int64_t)ms * 1000; }

int main(void)
{
    robot_cmd_t out;
    bool changed;
    cmd_hub_init();

    // Serial host drives by default
    robot_cmd_t c = mk(1, 500);
    cmd_hub_submit(CMD_SRC_SERIAL, &c);
    CHECK(cmd_hub_wait_command(&out, 0) && out.wheel1_speed == 500);
    CHECK(cmd_hub_wait_state_change(0, &changed) == 1 && changed);

    // BLE without takeover is ignored
    c = mk(4, 900);
    cmd_hub_submit(CMD_SRC_BLE, &c);
    CHECK(!cmd_hub_wait_command(&out, 0));
    CHECK(cmd_hub_host_state() == 1);

    // Takeover: forced idle first, then BLE commands are executed and serial is ignored
    cmd_hub_request_takeover(true);
    CHECK(cmd_hub_host_state() == 0);
    c = mk(4, 900);
    cmd_hub_submit(CMD_SRC_BLE, &c);
    CHECK(cmd_hub_wait_command(&out, 0) && out.wheel1_speed == 900 && out.host_state == 4);
    advance_ms(20);
    c = mk(1, 111);
    cmd_hub_submit(CMD_SRC_SERIAL, &c);
    CHECK(!cmd_hub_wait_command(&out, 0));
    CHECK(cmd_hub_host_state() == 4);

    // BLE goes quiet for longer than the timeout: next serial frame stops, then hands back
    advance_ms(CMD_HUB_SOURCE_TIMEOUT_MS + 10);
    c = mk(1, 222);
    cmd_hub_submit(CMD_SRC_SERIAL, &c);
    cmd_hub_status_t st;
    cmd_hub_get_status(&st);
    CHECK(!st.takeover && st.active == CMD_SRC_SERIAL);
    CHECK(cmd_hub_wait_command(&out, 0) && out.wheel1_speed == 222);

    // BLE disconnect while driving: immediate stop
    cmd_hub_request_takeover(true);
    c = mk(4, 800);
    cmd_hub_submit(CMD_SRC_BLE, &c);
    cmd_hub_wait_command(&out, 0);
    cmd_hub_source_lost(CMD_SRC_BLE);
    cmd_hub_get_status(&st);
    CHECK(!st.takeover && st.host_state == 0);
    // ...and the command consumer is woken with "stop" instead of waiting out its timeout
    CHECK(sems[0].count == 1);
    CHECK(!cmd_hub_wait_command(&out, 0));

    // BLE goes quiet and nobody else is sending: the consumer's own wait expires it
    cmd_hub_request_takeover(true);
    c = mk(4, 700);
    cmd_hub_submit(CMD_SRC_BLE, &c);
    CHECK(cmd_hub_wait_command(&out, 0) && out.wheel1_speed == 700);
    advance_ms(CMD_HUB_SOURCE_TIMEOUT_MS - 50);
    CHECK(!cmd_hub_wait_command(&out, 0));
    cmd_hub_get_status(&st);
    CHECK(st.takeover && st.host_state == 4);  // still within the timeout
    advance_ms(60);
    CHECK(!cmd_hub_wait_command(&out, 0));
    cmd_hub_get_status(&st);
    CHECK(!st.takeover && st.host_state == 0);
    CHECK(cmd_hub_wait_state_change(0, &changed) == 0 && changed);

    // Emergency stop latches: nothing is accepted until cleared, even from a new source
    c = mk(1, 300);
    cmd_hub_submit(CMD_SRC_SERIAL, &c);
    cmd_hub_set_estop(true);
    CHECK(cmd_hub_host_state() == 0 && cmd_hub_estopped());
    CHECK(!cmd_hub_wait_command(&out, 0));
    c = mk(1, 300);
    cmd_hub_submit(CMD_SRC_SERIAL, &c);
    CHECK(!cmd_hub_wait_command(&out, 0));
    CHECK(cmd_hub_host_state() == 0);
    cmd_hub_set_estop(false);
    CHECK(!cmd_hub_estopped());
    cmd_hub_submit(CMD_SRC_SERIAL, &c);
    CHECK(cmd_hub_wait_command(&out, 0) && out.wheel1_speed == 300);

    // Releasing control also passes through idle
    cmd_hub_request_takeover(true);
    c = mk(4, 100);
    cmd_hub_submit(CMD_SRC_BLE, &c);
    cmd_hub_request_takeover(false);
    CHECK(cmd_hub_host_state() == 0);

    printf(failures ? "%d failure(s)\n" : "cmd_hub: all checks passed\n", failures);
    return failures ? 1 : 0;
}
