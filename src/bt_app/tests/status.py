#!/usr/bin/env python3
import socket
import struct

MSP_STATUS_EX = 150
MSP_RC = 105

ARMING_FLAGS = [
    "NO_GYRO",
    "FAILSAFE",
    "RX_FAILSAFE",
    "NOT_DISARMED",
    "BOXFAILSAFE",
    "RUNAWAY_TAKEOFF",
    "CRASH_DETECTED",
    "THROTTLE",
    "ANGLE",
    "BOOT_GRACE_TIME",
    "NOPREARM",
    "LOAD",
    "CALIBRATING",
    "CLI",
    "CMS_MENU",
    "BST",
    "MSP",
    "PARALYZE",
    "GPS",
    "RESCUE_SW",
    "DSHOT_TELEM",
    "REBOOT_REQUIRED",
    "DSHOT_BITBANG",
    "ACC_CALIBRATION",
    "MOTOR_PROTOCOL",
    "CRASHFLIP",
    "ALTHOLD",
    "POSHOLD",
    "ARM_SWITCH",
]

SENSOR_BITS = [
    "ACC",
    "BARO",
    "MAG",
    "GPS",
    "RANGEFINDER",
    "GYRO",
    "OPTICALFLOW",
]


def msp_v1_frame(cmd: int, payload: bytes = b"") -> bytes:
    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b
    return b"$M<" + bytes([size, cmd]) + payload + bytes([checksum])


def recv_exact(sock: socket.socket, n: int) -> bytes:
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Socket closed")
        data += chunk
    return data


def read_msp_v1(sock: socket.socket):
    while True:
        if recv_exact(sock, 1) != b"$":
            continue
        if recv_exact(sock, 1) != b"M":
            continue

        direction = recv_exact(sock, 1)
        if direction not in (b">", b"!"):
            continue

        size = recv_exact(sock, 1)[0]
        cmd = recv_exact(sock, 1)[0]
        payload = recv_exact(sock, size)
        received_checksum = recv_exact(sock, 1)[0]

        checksum = size ^ cmd
        for b in payload:
            checksum ^= b

        if checksum != received_checksum:
            raise ValueError("Bad MSP checksum")

        if direction == b"!":
            raise RuntimeError(f"MSP error response for command {cmd}")

        return cmd, payload


def request(sock: socket.socket, cmd: int) -> bytes:
    sock.sendall(msp_v1_frame(cmd))
    while True:
        response_cmd, payload = read_msp_v1(sock)
        if response_cmd == cmd:
            return payload


def decode_rc(payload: bytes):
    count = len(payload) // 2
    return list(struct.unpack_from("<" + "H" * count, payload))


def decode_status_ex(payload: bytes):
    print("MSP_STATUS_EX raw:", payload.hex(" "))

    if len(payload) < 16:
        raise ValueError(f"Payload too short for MSP_STATUS_EX: {len(payload)} bytes")

    cycle_time_us = struct.unpack_from("<H", payload, 0)[0]
    i2c_errors = struct.unpack_from("<H", payload, 2)[0]
    sensors_mask = struct.unpack_from("<H", payload, 4)[0]
    box_mode_flags = struct.unpack_from("<I", payload, 6)[0]
    pid_profile = payload[10]

    # Betaflight STATUS_EX extra fields before the variable flight-mode bytes.
    cpu_load = struct.unpack_from("<H", payload, 11)[0]
    pid_profile_count = payload[13]
    rate_profile = payload[14]

    # Then Betaflight writes:
    #   byteCount
    #   flightModeFlags[byteCount]
    #   armingDisableFlagsCount
    #   uint32 armingDisableFlags
    flight_mode_byte_count = payload[15]
    off = 16 + flight_mode_byte_count

    if len(payload) < off + 5:
        raise ValueError(
            f"Cannot find arming flags. len={len(payload)}, "
            f"flight_mode_byte_count={flight_mode_byte_count}"
        )

    arming_disable_flag_count = payload[off]
    arming_disable_mask = struct.unpack_from("<I", payload, off + 1)[0]

    active_arming_flags = [
        name for bit, name in enumerate(ARMING_FLAGS)
        if arming_disable_mask & (1 << bit)
    ]

    active_sensors = [
        name for bit, name in enumerate(SENSOR_BITS)
        if sensors_mask & (1 << bit)
    ]

    return {
        "cycle_time_us": cycle_time_us,
        "i2c_errors": i2c_errors,
        "sensors_mask": f"0x{sensors_mask:04x}",
        "sensors": active_sensors,
        "box_mode_flags": f"0x{box_mode_flags:08x}",
        "pid_profile": pid_profile,
        "pid_profile_count": pid_profile_count,
        "rate_profile": rate_profile,
        "cpu_load_raw": cpu_load,
        "arming_disable_flag_count": arming_disable_flag_count,
        "arming_disable_mask": f"0x{arming_disable_mask:08x}",
        "arming_disable_flags": active_arming_flags,
    }


with socket.create_connection(("127.0.0.1", 5761), timeout=2.0) as sock:
    sock.settimeout(1.0)

    status_payload = request(sock, MSP_STATUS_EX)
    print("STATUS:")
    print(decode_status_ex(status_payload))

    rc_payload = request(sock, MSP_RC)
    print("RC channels:")
    print(decode_rc(rc_payload))