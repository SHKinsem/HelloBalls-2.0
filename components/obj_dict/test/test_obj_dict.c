/* Host tests for the object dictionary, SDO server and PDO packing.
 * Build and run: components/obj_dict/test/run_tests.sh */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "obj_dict.h"

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

// Test dictionary: the SDO frames in vectors.json are built against it
static uint8_t state = 1;
static int16_t speed = -1234;
static float gain = 1.5f;
static uint16_t limit = 500;
static char name[] = "HelloBalls";
static int triggers;
static uint8_t blob[300];

static uint32_t trigger(const od_entry_t *e, const uint8_t *data, size_t len)
{
    (void)e;
    (void)len;
    triggers += data[0];
    return OD_OK;
}

static uint32_t read_blob(const od_entry_t *e, uint32_t offset, uint8_t *buf, size_t cap,
                          size_t *len, uint32_t *total)
{
    (void)e;
    *total = sizeof(blob);
    if (offset > sizeof(blob)) {
        return OD_ABORT_LENGTH;
    }
    size_t n = sizeof(blob) - offset < cap ? sizeof(blob) - offset : cap;
    memcpy(buf, blob + offset, n);
    *len = n;
    return OD_OK;
}

static bool blocked = true;
static uint32_t gate(const od_entry_t *e, void *ctx)
{
    (void)ctx;
    return (blocked && (e->flags & OD_F_IDLE)) ? OD_ABORT_STATE : OD_OK;
}

static const od_entry_t entries[] = {
    {0x2000, 1, OD_U8, OD_R, OD_F_PDO, "状态", NULL, 0, 0, "A,B,\"C\"", &state, NULL, NULL},
    {0x2000, 2, OD_I16, OD_R, OD_F_PDO, "速度", "rpm", 0, 0, NULL, &speed, NULL, NULL},
    {0x2001, 1, OD_F32, OD_RW, OD_F_PDO | OD_F_RANGE, "增益", NULL, -10, 10, NULL, &gain, NULL, NULL},
    {0x2001, 2, OD_U16, OD_RW, OD_F_IDLE | OD_F_STORE, "上限", "rpm", 0, 0, NULL, &limit, NULL, NULL},
    {0x2002, 0, OD_STR, OD_R, 0, "名称", NULL, 0, 0, NULL, name, NULL, NULL},
    {0x2003, 1, OD_U8, OD_W, 0, "触发", NULL, 0, 0, NULL, NULL, NULL, trigger},
    {0x2004, 0, OD_DOMAIN, OD_R, 0, "数据", NULL, 0, 0, NULL, NULL, read_blob, NULL},
};
static const od_object_t objects[] = {
    {0x2000, "状态组"}, {0x2001, "参数"}, {0x2002, "名称"}, {0x2003, "动作"}, {0x2004, "大数据"},
};
static const od_dict_t dict = {entries, sizeof(entries) / sizeof(entries[0]),
                               objects, sizeof(objects) / sizeof(objects[0]), gate, NULL};

static size_t unhex(const char *s, uint8_t *out, size_t cap)
{
    size_t n = 0;
    while (s[0] && s[1] && n < cap) {
        unsigned v;
        sscanf(s, "%2x", &v);
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}

// Pull the next "key": "hex" value out of the vectors file
static const char *next_hex(const char *p, const char *key, char *out, size_t cap)
{
    char pat[16];
    snprintf(pat, sizeof(pat), "\"%s\": \"", key);
    p = strstr(p, pat);
    if (p == NULL) {
        return NULL;
    }
    p += strlen(pat);
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < cap) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return p;
}

static void test_vectors(const char *path)
{
    FILE *f = fopen(path, "rb");
    CHECK(f != NULL);
    if (f == NULL) {
        return;
    }
    static char text[16384];
    size_t n = fread(text, 1, sizeof(text) - 1, f);
    fclose(f);
    text[n] = '\0';

    const char *p = text;
    char req_hex[600], rsp_hex[600];
    int count = 0;
    while ((p = next_hex(p, "req", req_hex, sizeof(req_hex))) != NULL) {
        p = next_hex(p, "rsp", rsp_hex, sizeof(rsp_hex));
        CHECK(p != NULL);
        if (p == NULL) {
            break;
        }
        uint8_t req[300], want[300], got[300];
        size_t req_len = unhex(req_hex, req, sizeof(req));
        size_t want_len = unhex(rsp_hex, want, sizeof(want));
        size_t got_len = od_sdo_handle(&dict, req, req_len, got, 244);
        if (got_len != want_len || memcmp(got, want, want_len) != 0) {
            printf("FAIL vector %d (req %s)\n", count + 1, req_hex);
            failures++;
        }
        count++;
    }
    CHECK(count >= 10);
    CHECK(triggers == 1);
    CHECK(gain == 2.5f);
}

static void test_segments(void)
{
    for (size_t i = 0; i < sizeof(blob); i++) {
        blob[i] = (uint8_t)(i * 7);
    }
    uint8_t got[sizeof(blob)];
    uint32_t offset = 0;
    int rounds = 0;
    while (offset < sizeof(blob) && rounds < 20) {
        uint8_t req[10] = {1, OD_SDO_SEG_READ, 0x04, 0x20, 0, 4};
        memcpy(&req[6], &offset, 4);  // host is little-endian
        uint8_t rsp[64];
        size_t n = od_sdo_handle(&dict, req, sizeof(req), rsp, sizeof(rsp));
        CHECK(n > 10 && rsp[1] == OD_SDO_SEG_READ_RSP);
        uint32_t total;
        memcpy(&total, &rsp[6], 4);
        CHECK(total == sizeof(blob));
        size_t chunk = rsp[5] - 4;
        memcpy(&got[offset], &rsp[10], chunk);
        offset += (uint32_t)chunk;
        rounds++;
    }
    CHECK(offset == sizeof(blob) && memcmp(got, blob, sizeof(blob)) == 0);

    // A plain read of something longer than one frame is refused
    uint8_t req[6] = {2, OD_SDO_READ, 0x04, 0x20, 0, 0};
    uint8_t rsp[64];
    CHECK(od_sdo_handle(&dict, req, sizeof(req), rsp, sizeof(rsp)) == 10 && rsp[1] == OD_SDO_ABORT);
}

static void test_gate(void)
{
    uint8_t v[2] = {0x2c, 0x01};  // 300
    CHECK(od_write(&dict, 0x2001, 2, v, 2) == OD_ABORT_STATE && limit == 500);
    blocked = false;
    CHECK(od_write(&dict, 0x2001, 2, v, 2) == OD_OK && limit == 300);
    blocked = true;

    uint8_t nan[4] = {0x00, 0x00, 0xc0, 0x7f};
    CHECK(od_write(&dict, 0x2001, 1, nan, 4) == OD_ABORT_RANGE);
}

static void test_pdo(void)
{
    uint32_t map[] = {od_map(0x2000, 1, 8), od_map(0x2000, 2, 16), od_map(0x2001, 1, 32)};
    CHECK(od_pdo_validate(&dict, map, 3, 244) == OD_OK);
    CHECK(od_pdo_validate(&dict, map, 3, 10) == OD_ABORT_PDO_TOO_LONG);
    uint32_t bad_size[] = {od_map(0x2000, 2, 8)};
    CHECK(od_pdo_validate(&dict, bad_size, 1, 244) == OD_ABORT_LENGTH);
    uint32_t not_mappable[] = {od_map(0x2001, 2, 16)};
    CHECK(od_pdo_validate(&dict, not_mappable, 1, 244) == OD_ABORT_NOT_MAPPABLE);
    uint32_t absent[] = {od_map(0x2100, 1, 8)};
    CHECK(od_pdo_validate(&dict, absent, 1, 244) == OD_ABORT_NO_OBJECT);

    uint8_t buf[32];
    gain = 2.5f;
    size_t n = od_pdo_pack(&dict, 2, 9, 0x1234, map, 3, buf, sizeof(buf));
    uint8_t want[] = {2, 9, 0x34, 0x12, 1, 0x2e, 0xfb, 0x00, 0x00, 0x20, 0x40};
    CHECK(n == sizeof(want) && memcmp(buf, want, n) == 0);
}

static void test_describe(void)
{
    size_t need = od_describe(&dict, NULL, 0);
    char *json = malloc(need + 1);
    CHECK(od_describe(&dict, json, need + 1) == need);
    CHECK(strlen(json) == need);
    CHECK(strncmp(json, "{\"v\":1,\"o\":[{\"i\":8192,\"n\":\"状态组\",\"e\":[{\"s\":1,", 40) == 0);
    CHECK(strstr(json, "\"en\":[\"A\",\"B\",\"\\\"C\\\"\"]") != NULL);
    CHECK(strstr(json, "\"lo\":-10,\"hi\":10") != NULL);
    CHECK(strstr(json, "\"u\":\"rpm\"") != NULL);
    CHECK(json[need - 1] == '}' && json[need - 2] == ']');

    // A short buffer is filled and terminated, the full size is still reported
    char small[16];
    CHECK(od_describe(&dict, small, sizeof(small)) == need && strlen(small) == sizeof(small) - 1);
    free(json);

    CHECK(od_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
}

int main(int argc, char **argv)
{
    test_vectors(argc > 1 ? argv[1] : "vectors.json");
    test_segments();
    test_gate();
    test_pdo();
    test_describe();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("obj_dict: all tests passed\n");
    return 0;
}
