#!/usr/bin/env python3
"""Host tests for the BLE remote codec: compiles ble_remote_codec.c and checks it against vectors.json.

Usage: python3 components/ble_remote/test/run_tests.py
"""
import ctypes
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src", "ble_remote_codec.c")
INC = os.path.join(HERE, "..", "include")


class Cmd(ctypes.Structure):
    _fields_ = [("seq", ctypes.c_uint8), ("state", ctypes.c_uint8), ("flags", ctypes.c_uint8),
                ("wheel1_rpm", ctypes.c_int16), ("wheel2_rpm", ctypes.c_int16),
                ("tilt_angle", ctypes.c_int16), ("shoot_speed", ctypes.c_int16)]


class Action(ctypes.Structure):
    _fields_ = [("action", ctypes.c_uint8), ("param", ctypes.c_int16)]


class Status(ctypes.Structure):
    _fields_ = [("mcu_state", ctypes.c_uint8), ("host_state", ctypes.c_uint8), ("active_src", ctypes.c_uint8),
                ("flags", ctypes.c_uint8), ("last_seq", ctypes.c_uint8), ("wheel1_rpm", ctypes.c_int16),
                ("wheel2_rpm", ctypes.c_int16), ("uptime_ms", ctypes.c_uint32)]


KINDS = {"control": (Cmd, "control"), "action": (Action, "action"), "status": (Status, "status")}


def main():
    tmp = tempfile.mkdtemp()
    lib_path = os.path.join(tmp, "codec.so")
    subprocess.check_call(["cc", "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror", "-I", INC, SRC, "-o", lib_path])
    lib = ctypes.CDLL(lib_path)
    with open(os.path.join(HERE, "vectors.json")) as f:
        vectors = json.load(f)

    failures = 0
    for kind, (struct, name) in KINDS.items():
        enc = getattr(lib, f"ble_remote_encode_{name}")
        dec = getattr(lib, f"ble_remote_decode_{name}")
        for v in vectors[kind]:
            expected = bytes.fromhex(v["hex"].replace(" ", ""))
            obj = struct(**v["fields"])
            buf = (ctypes.c_uint8 * 32)()
            n = enc(ctypes.byref(obj), buf, 32)
            got = bytes(buf[:n])
            if got != expected:
                print(f"FAIL encode {kind}: {got.hex()} != {expected.hex()}")
                failures += 1
            out = struct()
            raw = (ctypes.c_uint8 * len(expected)).from_buffer_copy(expected)
            if not dec(raw, len(expected), ctypes.byref(out)):
                print(f"FAIL decode {kind}: rejected {expected.hex()}")
                failures += 1
                continue
            for field, value in v["fields"].items():
                if getattr(out, field) != value:
                    print(f"FAIL decode {kind}.{field}: {getattr(out, field)} != {value}")
                    failures += 1

    for v in vectors["invalid"]:
        data = bytes.fromhex(v["hex"].replace(" ", ""))
        struct, name = KINDS[v["kind"]]
        dec = getattr(lib, f"ble_remote_decode_{name}")
        raw = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        if dec(raw, len(data), ctypes.byref(struct())):
            print(f"FAIL invalid {v['kind']} accepted ({v['why']})")
            failures += 1

    total = sum(len(vectors[k]) for k in KINDS) + len(vectors["invalid"])
    print(f"{total}/{total} vectors ok" if not failures else f"{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
