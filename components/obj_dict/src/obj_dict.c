#include "obj_dict.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static uint32_t key_of(uint16_t index, uint8_t sub)
{
    return ((uint32_t)index << 8) | sub;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

size_t od_type_size(uint8_t type)
{
    switch (type) {
    case OD_BOOL:
    case OD_I8:
    case OD_U8:
        return 1;
    case OD_I16:
    case OD_U16:
        return 2;
    case OD_I32:
    case OD_U32:
    case OD_F32:
        return 4;
    default:
        return 0;
    }
}

const od_entry_t *od_find(const od_dict_t *d, uint16_t index, uint8_t sub)
{
    if (d == NULL || d->entries == NULL) {
        return NULL;
    }
    uint32_t key = key_of(index, sub);
    size_t lo = 0, hi = d->count;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        uint32_t k = key_of(d->entries[mid].index, d->entries[mid].sub);
        if (k == key) {
            return &d->entries[mid];
        }
        if (k < key) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return NULL;
}

static bool index_exists(const od_dict_t *d, uint16_t index)
{
    for (size_t i = 0; i < d->count; i++) {
        if (d->entries[i].index == index) {
            return true;
        }
    }
    return false;
}

static uint32_t missing(const od_dict_t *d, uint16_t index)
{
    return index_exists(d, index) ? OD_ABORT_NO_SUB : OD_ABORT_NO_OBJECT;
}

bool od_decode_number(uint8_t type, const uint8_t *p, size_t len, double *out)
{
    if (od_type_size(type) == 0 || len != od_type_size(type)) {
        return false;
    }
    switch (type) {
    case OD_BOOL:
    case OD_U8:  *out = p[0]; break;
    case OD_I8:  *out = (int8_t)p[0]; break;
    case OD_U16: *out = get_u16(p); break;
    case OD_I16: *out = (int16_t)get_u16(p); break;
    case OD_U32: *out = get_u32(p); break;
    case OD_I32: *out = (int32_t)get_u32(p); break;
    case OD_F32: {
        uint32_t u = get_u32(p);
        float f;
        memcpy(&f, &u, sizeof(f));
        *out = f;
        break;
    }
    default:
        return false;
    }
    return true;
}

size_t od_encode_number(uint8_t type, double v, uint8_t *buf, size_t cap)
{
    size_t n = od_type_size(type);
    if (n == 0 || cap < n) {
        return 0;
    }
    switch (type) {
    case OD_BOOL: buf[0] = v != 0; break;
    case OD_U8:   buf[0] = (uint8_t)v; break;
    case OD_I8:   buf[0] = (uint8_t)(int8_t)v; break;
    case OD_U16:  put_u16(buf, (uint16_t)v); break;
    case OD_I16:  put_u16(buf, (uint16_t)(int16_t)v); break;
    case OD_U32:  put_u32(buf, (uint32_t)v); break;
    case OD_I32:  put_u32(buf, (uint32_t)(int32_t)v); break;
    case OD_F32: {
        float f = (float)v;
        uint32_t u;
        memcpy(&u, &f, sizeof(u));
        put_u32(buf, u);
        break;
    }
    default:
        return 0;
    }
    return n;
}

static uint32_t read_entry(const od_entry_t *e, uint32_t offset, uint8_t *buf, size_t cap,
                           size_t *len, uint32_t *total)
{
    *len = 0;
    *total = 0;
    if (!(e->access & OD_R)) {
        return OD_ABORT_WRITE_ONLY;
    }
    if (e->read) {
        return e->read(e, offset, buf, cap, len, total);
    }
    if (e->ptr == NULL) {
        return OD_ABORT_GENERAL;
    }
    size_t size = od_type_size(e->type);
    if (size == 0) {
        // STR backed by a NUL-terminated buffer
        const char *s = (const char *)e->ptr;
        size_t n = strlen(s);
        *total = (uint32_t)n;
        if (offset > n) {
            return OD_ABORT_LENGTH;
        }
        size_t chunk = n - offset < cap ? n - offset : cap;
        memcpy(buf, s + offset, chunk);
        *len = chunk;
        return OD_OK;
    }
    if (offset != 0 || cap < size) {
        return OD_ABORT_LENGTH;
    }
    memcpy(buf, e->ptr, size);
    *len = size;
    *total = (uint32_t)size;
    return OD_OK;
}

uint32_t od_read(const od_dict_t *d, uint16_t index, uint8_t sub, uint32_t offset,
                 uint8_t *buf, size_t cap, size_t *len, uint32_t *total)
{
    const od_entry_t *e = od_find(d, index, sub);
    if (e == NULL) {
        *len = 0;
        *total = 0;
        return missing(d, index);
    }
    return read_entry(e, offset, buf, cap, len, total);
}

uint32_t od_write(const od_dict_t *d, uint16_t index, uint8_t sub, const uint8_t *data, size_t len)
{
    const od_entry_t *e = od_find(d, index, sub);
    if (e == NULL) {
        return missing(d, index);
    }
    if (!(e->access & OD_W)) {
        return OD_ABORT_READ_ONLY;
    }
    size_t size = od_type_size(e->type);
    if (size != 0) {
        double v;
        if (!od_decode_number(e->type, data, len, &v)) {
            return OD_ABORT_LENGTH;
        }
        if (e->type == OD_BOOL && data[0] > 1) {
            return OD_ABORT_RANGE;
        }
        if (v != v) {  // NaN
            return OD_ABORT_RANGE;
        }
        if ((e->flags & OD_F_RANGE) && (v < e->min || v > e->max)) {
            return OD_ABORT_RANGE;
        }
    } else if (e->write == NULL) {
        return OD_ABORT_READ_ONLY;  // STR/DOMAIN writes need a hook
    }
    if (d->gate) {
        uint32_t rc = d->gate(e, d->gate_ctx);
        if (rc != OD_OK) {
            return rc;
        }
    }

    uint8_t old[4];
    if (size != 0 && e->ptr != NULL) {
        memcpy(old, e->ptr, size);
        memcpy(e->ptr, data, size);
    }
    if (e->write) {
        uint32_t rc = e->write(e, data, len);
        if (rc != OD_OK) {
            if (size != 0 && e->ptr != NULL) {
                memcpy(e->ptr, old, size);
            }
            return rc;
        }
    }
    return OD_OK;
}

static size_t sdo_header(uint8_t *rsp, uint8_t seq, uint8_t cmd, uint16_t index, uint8_t sub, size_t len)
{
    rsp[0] = seq;
    rsp[1] = cmd;
    put_u16(&rsp[2], index);
    rsp[4] = sub;
    rsp[5] = (uint8_t)len;
    return OD_SDO_HEADER + len;
}

static size_t sdo_abort(uint8_t *rsp, uint8_t seq, uint16_t index, uint8_t sub, uint32_t code)
{
    put_u32(&rsp[OD_SDO_HEADER], code);
    return sdo_header(rsp, seq, OD_SDO_ABORT, index, sub, 4);
}

size_t od_sdo_handle(const od_dict_t *d, const uint8_t *req, size_t req_len, uint8_t *rsp, size_t rsp_cap)
{
    if (req == NULL || rsp == NULL || req_len < OD_SDO_HEADER || rsp_cap < OD_SDO_HEADER + 8) {
        return 0;
    }
    uint8_t seq = req[0];
    uint8_t cmd = req[1];
    uint16_t index = get_u16(&req[2]);
    uint8_t sub = req[4];
    size_t dlen = req[5];
    const uint8_t *data = &req[OD_SDO_HEADER];
    if (OD_SDO_HEADER + dlen != req_len) {
        return sdo_abort(rsp, seq, index, sub, OD_ABORT_LENGTH);
    }
    // the length byte limits a response to 255 data bytes
    size_t room = rsp_cap - OD_SDO_HEADER;
    if (room > 255) {
        room = 255;
    }

    size_t len = 0;
    uint32_t total = 0;
    uint32_t rc;
    switch (cmd) {
    case OD_SDO_READ: {
        const od_entry_t *e = od_find(d, index, sub);
        if (e != NULL && od_type_size(e->type) == 0) {
            // A short string fits a plain read; longer ones need segments
            rc = read_entry(e, 0, &rsp[OD_SDO_HEADER], room, &len, &total);
            if (rc == OD_OK && total > len) {
                rc = OD_ABORT_LENGTH;
            }
        } else {
            rc = od_read(d, index, sub, 0, &rsp[OD_SDO_HEADER], room, &len, &total);
        }
        if (rc != OD_OK) {
            return sdo_abort(rsp, seq, index, sub, rc);
        }
        return sdo_header(rsp, seq, OD_SDO_READ_RSP, index, sub, len);
    }
    case OD_SDO_SEG_READ:
        if (dlen != 4) {
            return sdo_abort(rsp, seq, index, sub, OD_ABORT_LENGTH);
        }
        rc = od_read(d, index, sub, get_u32(data), &rsp[OD_SDO_HEADER + 4], room - 4, &len, &total);
        if (rc != OD_OK) {
            return sdo_abort(rsp, seq, index, sub, rc);
        }
        put_u32(&rsp[OD_SDO_HEADER], total);
        return sdo_header(rsp, seq, OD_SDO_SEG_READ_RSP, index, sub, 4 + len);
    case OD_SDO_WRITE:
        rc = od_write(d, index, sub, data, dlen);
        if (rc != OD_OK) {
            return sdo_abort(rsp, seq, index, sub, rc);
        }
        // Answer with the value now in effect (write-only entries echo nothing)
        rc = od_read(d, index, sub, 0, &rsp[OD_SDO_HEADER], room, &len, &total);
        if (rc != OD_OK || total > len) {
            len = 0;
        }
        return sdo_header(rsp, seq, OD_SDO_WRITE_RSP, index, sub, len);
    default:
        return sdo_abort(rsp, seq, index, sub, OD_ABORT_TOGGLE);
    }
}

uint32_t od_pdo_validate(const od_dict_t *d, const uint32_t *map, size_t n, size_t max_payload)
{
    if (n > OD_PDO_MAX_MAP) {
        return OD_ABORT_PDO_TOO_LONG;
    }
    size_t total = 0;
    for (size_t i = 0; i < n; i++) {
        uint16_t index = (uint16_t)(map[i] >> 16);
        uint8_t sub = (uint8_t)(map[i] >> 8);
        uint8_t bits = (uint8_t)map[i];
        const od_entry_t *e = od_find(d, index, sub);
        if (e == NULL) {
            return missing(d, index);
        }
        if (!(e->access & OD_R) || !(e->flags & OD_F_PDO)) {
            return OD_ABORT_NOT_MAPPABLE;
        }
        size_t size = od_type_size(e->type);
        if (size == 0 || bits != size * 8) {
            return OD_ABORT_LENGTH;
        }
        total += size;
    }
    if (OD_PDO_HEADER + total > max_payload) {
        return OD_ABORT_PDO_TOO_LONG;
    }
    return OD_OK;
}

size_t od_pdo_pack(const od_dict_t *d, uint8_t pdo, uint8_t seq, uint16_t t_ms,
                   const uint32_t *map, size_t n, uint8_t *buf, size_t cap)
{
    if (buf == NULL || cap < OD_PDO_HEADER) {
        return 0;
    }
    buf[0] = pdo;
    buf[1] = seq;
    put_u16(&buf[2], t_ms);
    size_t pos = OD_PDO_HEADER;
    for (size_t i = 0; i < n; i++) {
        size_t len = 0;
        uint32_t total = 0;
        uint32_t rc = od_read(d, (uint16_t)(map[i] >> 16), (uint8_t)(map[i] >> 8), 0,
                              &buf[pos], cap - pos, &len, &total);
        if (rc != OD_OK || len * 8 != (map[i] & 0xFF)) {
            return 0;
        }
        pos += len;
    }
    return pos;
}

// Bounded JSON writer: counts everything, stores what fits
typedef struct {
    char *buf;
    size_t cap;
    size_t len;
} jw_t;

static void jw_raw(jw_t *w, const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (w->buf != NULL && w->len + 1 < w->cap) {
            w->buf[w->len] = s[i];
        }
        w->len++;
    }
    if (w->buf != NULL && w->cap > 0) {
        w->buf[w->len < w->cap ? w->len : w->cap - 1] = '\0';
    }
}

static void jw_fmt(jw_t *w, const char *fmt, ...)
{
    char tmp[48];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n > 0) {
        jw_raw(w, tmp, (size_t)n < sizeof(tmp) ? (size_t)n : sizeof(tmp) - 1);
    }
}

static void jw_str(jw_t *w, const char *s, size_t n)
{
    jw_raw(w, "\"", 1);
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '"' || c == '\\') {
            jw_raw(w, "\\", 1);
            jw_raw(w, (const char *)&c, 1);
        } else if (c < 0x20) {
            jw_fmt(w, "\\u%04x", c);
        } else {
            jw_raw(w, (const char *)&c, 1);
        }
    }
    jw_raw(w, "\"", 1);
}

static void jw_num(jw_t *w, float v)
{
    jw_fmt(w, "%.7g", (double)v);
}

static void describe_entry(jw_t *w, const od_entry_t *e)
{
    jw_fmt(w, "{\"s\":%u,\"n\":", e->sub);
    jw_str(w, e->name ? e->name : "", e->name ? strlen(e->name) : 0);
    jw_fmt(w, ",\"t\":%u,\"a\":%u,\"f\":%u", e->type, e->access, e->flags);
    if (e->unit) {
        jw_raw(w, ",\"u\":", 5);
        jw_str(w, e->unit, strlen(e->unit));
    }
    if (e->flags & OD_F_RANGE) {
        jw_raw(w, ",\"lo\":", 6);
        jw_num(w, e->min);
        jw_raw(w, ",\"hi\":", 6);
        jw_num(w, e->max);
    }
    if (e->enums) {
        jw_raw(w, ",\"en\":[", 7);
        const char *p = e->enums;
        bool first = true;
        while (true) {
            const char *comma = strchr(p, ',');
            size_t n = comma ? (size_t)(comma - p) : strlen(p);
            if (!first) {
                jw_raw(w, ",", 1);
            }
            jw_str(w, p, n);
            first = false;
            if (!comma) {
                break;
            }
            p = comma + 1;
        }
        jw_raw(w, "]", 1);
    }
    jw_raw(w, "}", 1);
}

size_t od_describe(const od_dict_t *d, char *buf, size_t cap)
{
    jw_t w = {buf, cap, 0};
    jw_raw(&w, "{\"v\":1,\"o\":[", 12);
    bool first_obj = true;
    size_t i = 0;
    while (i < d->count) {
        uint16_t index = d->entries[i].index;
        const char *name = "";
        for (size_t k = 0; k < d->object_count; k++) {
            if (d->objects[k].index == index) {
                name = d->objects[k].name;
                break;
            }
        }
        if (!first_obj) {
            jw_raw(&w, ",", 1);
        }
        first_obj = false;
        jw_fmt(&w, "{\"i\":%u,\"n\":", index);
        jw_str(&w, name, strlen(name));
        jw_raw(&w, ",\"e\":[", 6);
        bool first = true;
        for (; i < d->count && d->entries[i].index == index; i++) {
            if (!first) {
                jw_raw(&w, ",", 1);
            }
            first = false;
            describe_entry(&w, &d->entries[i]);
        }
        jw_raw(&w, "]}", 2);
    }
    jw_raw(&w, "]}", 2);
    return w.len;
}

uint32_t od_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1));
        }
    }
    return ~crc;
}
