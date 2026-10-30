/*
 * CANopen-style object dictionary: entries addressed by index:subindex, with
 * types, access rights, ranges and flags, plus an SDO server and PDO packing.
 * Pure C with no ESP-IDF dependency so it can be unit tested on a PC; the
 * application supplies the table and the hooks that touch the hardware.
 *
 * All values are little-endian on the wire. Design doc:
 * "HelloBalls 蓝牙对象字典设计（类 CANopen）".
 */
#ifndef OBJ_DICT_H
#define OBJ_DICT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Data types (CANopen codes)
typedef enum {
    OD_BOOL   = 0x01,
    OD_I8     = 0x02,
    OD_I16    = 0x03,
    OD_I32    = 0x04,
    OD_U8     = 0x05,
    OD_U16    = 0x06,
    OD_U32    = 0x07,
    OD_F32    = 0x08,
    OD_STR    = 0x09,
    OD_DOMAIN = 0x0F,
} od_type_t;

// Access rights
#define OD_R   0x01
#define OD_W   0x02
#define OD_RW  (OD_R | OD_W)

// Flags
#define OD_F_PDO      0x01  // may be mapped into a TPDO
#define OD_F_STORE    0x02  // saved to flash by 0x1010
#define OD_F_TAKEOVER 0x04  // writable only while the BLE remote has taken control
#define OD_F_IDLE     0x08  // writable only while the motors are stopped and the host state is IDLE
#define OD_F_CONFIRM  0x10  // the app asks the user to confirm before writing
#define OD_F_RANGE    0x80  // min/max are valid (numeric types)

// SDO abort codes (CANopen subset)
#define OD_OK                  0x00000000u
#define OD_ABORT_TOGGLE        0x05030000u  // unknown SDO command
#define OD_ABORT_WRITE_ONLY    0x06010001u
#define OD_ABORT_READ_ONLY     0x06010002u
#define OD_ABORT_NO_OBJECT     0x06020000u
#define OD_ABORT_NOT_MAPPABLE  0x06040041u
#define OD_ABORT_PDO_TOO_LONG  0x06040042u
#define OD_ABORT_LENGTH        0x06070010u
#define OD_ABORT_NO_SUB        0x06090011u
#define OD_ABORT_RANGE         0x06090030u
#define OD_ABORT_GENERAL       0x08000000u
#define OD_ABORT_STORE         0x08000020u
#define OD_ABORT_STATE         0x08000022u

typedef struct od_entry od_entry_t;

// Read a value (or, for STR/DOMAIN, a slice from offset). Writes up to cap bytes to buf.
// Sets *len to the bytes written and *total to the full size. Returns OD_OK or an abort code.
typedef uint32_t (*od_read_fn)(const od_entry_t *e, uint32_t offset, uint8_t *buf, size_t cap,
                               size_t *len, uint32_t *total);
// Apply a write that already passed type, range and gate checks. Returns OD_OK or an abort code.
typedef uint32_t (*od_write_fn)(const od_entry_t *e, const uint8_t *data, size_t len);

struct od_entry {
    uint16_t index;
    uint8_t sub;
    uint8_t type;       // od_type_t
    uint8_t access;     // OD_R / OD_W
    uint8_t flags;      // OD_F_*
    const char *name;
    const char *unit;   // NULL when unitless
    float min, max;     // used when OD_F_RANGE is set
    const char *enums;  // comma-separated labels for value 0,1,2..., or NULL
    void *ptr;          // backing variable for plain entries (read and/or write)
    od_read_fn read;    // optional, takes precedence over ptr
    od_write_fn write;  // optional, called after ptr is updated (or instead when ptr is NULL)
};

typedef struct {
    uint16_t index;
    const char *name;
} od_object_t;

// Returns OD_OK when the write is allowed in the current device state, else an abort code.
typedef uint32_t (*od_gate_fn)(const od_entry_t *e, void *ctx);

typedef struct {
    const od_entry_t *entries;  // sorted by index, then sub
    size_t count;
    const od_object_t *objects; // names of each index, sorted
    size_t object_count;
    od_gate_fn gate;
    void *gate_ctx;
} od_dict_t;

// Size in bytes of a fixed-size type, 0 for STR/DOMAIN
size_t od_type_size(uint8_t type);

const od_entry_t *od_find(const od_dict_t *d, uint16_t index, uint8_t sub);

// Read a whole value (fixed types) or a slice (STR/DOMAIN)
uint32_t od_read(const od_dict_t *d, uint16_t index, uint8_t sub, uint32_t offset,
                 uint8_t *buf, size_t cap, size_t *len, uint32_t *total);

// Check access, length, range and device state, then apply
uint32_t od_write(const od_dict_t *d, uint16_t index, uint8_t sub, const uint8_t *data, size_t len);

// Value conversion helpers (numeric types only)
bool od_decode_number(uint8_t type, const uint8_t *data, size_t len, double *out);
size_t od_encode_number(uint8_t type, double v, uint8_t *buf, size_t cap);

/*
 * SDO frame (request and response): seq u8 | cmd u8 | index u16 | sub u8 | len u8 | data[len]
 */
#define OD_SDO_HEADER       6
#define OD_SDO_READ         0x40  // -> 0x43 + value
#define OD_SDO_READ_RSP     0x43
#define OD_SDO_SEG_READ     0x41  // data: offset u32 -> 0x42 + total u32 + slice
#define OD_SDO_SEG_READ_RSP 0x42
#define OD_SDO_WRITE        0x23  // data: value -> 0x60 + value read back
#define OD_SDO_WRITE_RSP    0x60
#define OD_SDO_ABORT        0x80  // data: abort u32

// Handle one request and build the response into rsp. Returns the response length (0 if req is too short).
size_t od_sdo_handle(const od_dict_t *d, const uint8_t *req, size_t req_len, uint8_t *rsp, size_t rsp_cap);

/*
 * TPDO: the device sends pdo u8 | seq u8 | t_ms u16 | values of the mapped entries.
 * A mapping word is index << 16 | sub << 8 | bit length, as in CANopen.
 */
#define OD_PDO_HEADER   4
#define OD_PDO_MAX_MAP  8

static inline uint32_t od_map(uint16_t index, uint8_t sub, uint8_t bits)
{
    return ((uint32_t)index << 16) | ((uint32_t)sub << 8) | bits;
}

// Check that every mapping word names an existing, readable, PDO-mappable entry of that size,
// and that the whole PDO fits into max_payload bytes.
uint32_t od_pdo_validate(const od_dict_t *d, const uint32_t *map, size_t n, size_t max_payload);

// Pack the mapped values after the header. Returns the frame length, 0 on error.
size_t od_pdo_pack(const od_dict_t *d, uint8_t pdo, uint8_t seq, uint16_t t_ms,
                   const uint32_t *map, size_t n, uint8_t *buf, size_t cap);

/*
 * Descriptor: the whole dictionary as compact JSON, read by the app through
 * 0x1021 (segmented) and cached by its CRC32 (0x1022). Format:
 * {"v":1,"o":[{"i":8705,"n":"左轮","e":[{"s":1,"n":"实测转速","t":3,"a":1,"f":1,"u":"rpm","lo":-9000,"hi":9000,"en":["a","b"]}]}]}
 * Returns the JSON length (excluding the terminator); writes nothing past cap.
 * Call with buf NULL to get the required size.
 */
size_t od_describe(const od_dict_t *d, char *buf, size_t cap);

uint32_t od_crc32(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // OBJ_DICT_H
