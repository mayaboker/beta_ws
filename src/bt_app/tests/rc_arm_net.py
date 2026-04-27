#!/usr/bin/env python3

import socket
import struct
import time

MSP_API_VERSION = 1
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_STATUS_EX = 150
MSP_SET_RAW_RC = 200

SITL_HOST = "127.0.0.1"
SITL_PORT = 5761


def msp_v1_frame(cmd: int, payload: bytes = b"") -> bytes:
    if not 0 <= cmd <= 255:
        raise ValueError("MSP v1 command must be 0..255")
    if len(payload) > 255:
        raise ValueError("MSP v1 payload too large")

    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b

    return b"$M<" + bytes([size, cmd]) + payload + bytes([checksum])


def recv_exact(sock: socket.socket, n: int, timeout: float = 0.5) -> bytes:
    sock.settimeout(timeout)

    data = b""
    while len(data) < n:
        try:
            chunk = sock.recv(n - len(data))
        except socket.timeout:
            raise TimeoutError(f"Timeout while waiting for {n} bytes")

        if not chunk:
            raise ConnectionError("Socket closed by Betaflight SITL")

        data += chunk

    return data


def read_msp_v1(sock: socket.socket, timeout: float = 0.5):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        remaining = max(0.001, deadline - time.monotonic())

        try:
            b = recv_exact(sock, 1, timeout=remaining)
        except TimeoutError:
            continue

        if b != b"$":
            continue

        remaining = max(0.001, deadline - time.monotonic())

        try:
            if recv_exact(sock, 1, timeout=remaining) != b"M":
                continue

            direction = recv_exact(sock, 1, timeout=remaining)
            if direction not in (b">", b"!"):
                continue

            size = recv_exact(sock, 1, timeout=remaining)[0]
            cmd = recv_exact(sock, 1, timeout=remaining)[0]
            payload = recv_exact(sock, size, timeout=remaining)
            received_checksum = recv_exact(sock, 1, timeout=remaining)[0]

        except TimeoutError:
            raise TimeoutError("Incomplete MSP frame")

        checksum = size ^ cmd
        for x in payload:
            checksum ^= x

        if checksum != received_checksum:
            raise ValueError(
                f"Bad MSP checksum: got 0x{received_checksum:02x}, "
                f"expected 0x{checksum:02x}"
            )

        if direction == b"!":
            raise RuntimeError(f"MSP error response for command {cmd}")

        return cmd, payload

    raise TimeoutError("No MSP response")


def request_msp(sock: socket.socket, cmd: int, timeout: float = 0.5) -> bytes:
    sock.sendall(msp_v1_frame(cmd))

    while True:
        rcmd, payload = read_msp_v1(sock, timeout=timeout)
        if rcmd == cmd:
            return payload


def send_raw_rc(sock: socket.socket, channels):
    """
    channels: 8 RC values, normally 1000..2000 microseconds.

    Default Betaflight AETR1234:
      CH1 roll
      CH2 pitch
      CH3 throttle
      CH4 yaw
      CH5 AUX1 / ARM
      CH6 AUX2
      CH7 AUX3
      CH8 AUX4
    """
    if len(channels) != 8:
        raise ValueError("MSP_SET_RAW_RC example uses exactly 8 channels")

    payload = struct.pack("<8H", *channels)
    sock.sendall(msp_v1_frame(MSP_SET_RAW_RC, payload))

    # Betaflight may send an ACK. Try to consume it, but do not block RC stream.
    old_timeout = sock.gettimeout()
    sock.settimeout(0.005)

    try:
        try:
            read_msp_v1(sock, timeout=0.005)
        except TimeoutError:
            pass
    finally:
        sock.settimeout(old_timeout)


def read_rc(sock: socket.socket):
    payload = request_msp(sock, MSP_RC)
    count = len(payload) // 2
    return list(struct.unpack_from("<" + "H" * count, payload))


def read_attitude(sock: socket.socket):
    payload = request_msp(sock, MSP_ATTITUDE)
    roll_x10, pitch_x10, heading = struct.unpack_from("<hhh", payload)

    return {
        "roll_deg": roll_x10 / 10.0,
        "pitch_deg": pitch_x10 / 10.0,
        "heading_deg": heading,
    }


def read_altitude(sock: socket.socket):
    payload = request_msp(sock, MSP_ALTITUDE)

    # Common MSP_ALTITUDE layout:
    # int32 altitude_cm, int16 vario_cm_s
    altitude_cm, vario_cm_s = struct.unpack_from("<ih", payload)

    return {
        "altitude_m": altitude_cm / 100.0,
        "vertical_speed_m_s": vario_cm_s / 100.0,
    }


def read_raw_imu(sock: socket.socket):
    payload = request_msp(sock, MSP_RAW_IMU)
    values = struct.unpack_from("<9h", payload)

    return {
        "acc": values[0:3],
        "gyro": values[3:6],
        "mag": values[6:9],
    }


ARMING_DISABLE_FLAGS = {
    0:  "NO_GYRO",
    1:  "FAILSAFE",
    2:  "RX_FAILSAFE",
    3:  "NOT_DISARMED",
    4:  "BOXFAILSAFE",
    5:  "RUNAWAY_TAKEOFF",
    6:  "CRASH_DETECTED",
    7:  "THROTTLE",
    8:  "ANGLE",
    9:  "BOOT_GRACE_TIME",
    10: "NOPREARM",
    11: "LOAD",
    12: "CALIBRATING",
    13: "CLI",
    14: "CMS_MENU",
    15: "BST",
    16: "MSP",
    17: "PARALYZE",
    18: "GPS",
    19: "RESCUE_SW",
    20: "RPMFILTER",
    21: "REBOOT_REQUIRED",
    22: "DSHOT_BITBANG",
    23: "ACC_CALIBRATION",
    24: "MOTOR_PROTOCOL",
    25: "ARMING_DISABLED_ARM_SWITCH",
    26: "ALTITUDE",
    27: "POSITION",
    28: "ARM_SWITCH",
}


def decode_arming_mask(mask: int):
    return [
        name
        for bit, name in ARMING_DISABLE_FLAGS.items()
        if mask & (1 << bit)
    ]


def read_status_ex(sock: socket.socket):
    payload = request_msp(sock, MSP_STATUS_EX)

    if len(payload) < 16:
        raise ValueError(f"MSP_STATUS_EX payload too short: {len(payload)} bytes")

    cycle_time_us = struct.unpack_from("<H", payload, 0)[0]
    i2c_errors = struct.unpack_from("<H", payload, 2)[0]
    sensors_mask = struct.unpack_from("<H", payload, 4)[0]
    box_mode_flags = struct.unpack_from("<I", payload, 6)[0]
    pid_profile = payload[10]
    cpu_load = struct.unpack_from("<H", payload, 11)[0]
    pid_profile_count = payload[13]
    rate_profile = payload[14]

    flight_mode_byte_count = payload[15]
    off = 16 + flight_mode_byte_count

    if len(payload) < off + 5:
        raise ValueError(
            f"Cannot decode arming flags. payload_len={len(payload)}, "
            f"flight_mode_byte_count={flight_mode_byte_count}"
        )

    arming_disable_flag_count = payload[off]
    arming_disable_mask = struct.unpack_from("<I", payload, off + 1)[0]

    return {
        "cycle_time_us": cycle_time_us,
        "i2c_errors": i2c_errors,
        "sensors_mask": f"0x{sensors_mask:04x}",
        "box_mode_flags": f"0x{box_mode_flags:08x}",
        "pid_profile": pid_profile,
        "pid_profile_count": pid_profile_count,
        "rate_profile": rate_profile,
        "cpu_load_raw": cpu_load,
        "arming_disable_flag_count": arming_disable_flag_count,
        "arming_disable_mask": f"0x{arming_disable_mask:08x}",
        "arming_disable_flags": decode_arming_mask(arming_disable_mask),
    }


def send_for(duration_s: float, sock: socket.socket, channels, rate_hz: float = 50):
    dt = 1.0 / rate_hz
    end = time.monotonic() + duration_s

    while time.monotonic() < end:
        send_raw_rc(sock, channels)
        time.sleep(dt)

def wait_until_not_calibrating(sock, timeout_s=15.0):
    end = time.monotonic() + timeout_s

    while time.monotonic() < end:
        status = read_status_ex(sock)
        flags = status["arming_disable_flags"]

        print("Waiting for calibration:", flags)

        if "CALIBRATING" not in flags:
            return status

        # Keep RX valid and arm switch LOW while waiting
        send_raw_rc(sock, [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000])
        time.sleep(0.1)

    raise TimeoutError("CALIBRATING did not clear")

def main():
    # Default AETR1234:
    # CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw,
    # CH5 AUX1/ARM, CH6 AUX2, CH7 AUX3, CH8 AUX4

    disarmed = [
        1500,  # roll
        1500,  # pitch
        1000,  # throttle low
        1500,  # yaw
        1000,  # AUX1 arm low
        1000,
        1000,
        1000,
    ]

    armed_low_throttle = [
        1500,  # roll
        1500,  # pitch
        1000,  # throttle low
        1500,  # yaw
        1900,  # AUX1 arm high
        1000,
        1000,
        1000,
    ]

    armed_small_throttle = [
        1500,  # roll
        1500,  # pitch
        1050,  # small throttle test
        1500,  # yaw
        1900,  # AUX1 arm high
        1000,
        1000,
        1000,
    ]

    with socket.create_connection((SITL_HOST, SITL_PORT), timeout=2.0) as sock:
        sock.settimeout(0.05)

        print(f"Connected to Betaflight SITL MSP at {SITL_HOST}:{SITL_PORT}")

        # Wait for SITL boot/calibration.
        time.sleep(2.0)

        api = request_msp(sock, MSP_API_VERSION)
        print("MSP API raw:", api.hex(" "))

        print("Initial status:", read_status_ex(sock))
        print("Initial RC:", read_rc(sock))

        print("Attitude:", read_attitude(sock))
        print("Altitude:", read_altitude(sock))
        print("Raw IMU:", read_raw_imu(sock))

        # Wait until CALIBRATING clears while keeping RC valid.
        print("Waiting for CALIBRATING to clear...")
        status = wait_until_not_calibrating(sock, timeout_s=15.0)
        print("Calibration cleared:", status)
              
        print("Sending disarmed neutral...")
        send_for(2.0, sock, disarmed, rate_hz=50)

        print("RC after disarmed frames:", read_rc(sock))
        print("Status after disarmed frames:", read_status_ex(sock))

        print("ARM: AUX1 high, throttle low...")
        send_for(20.0, sock, armed_low_throttle, rate_hz=50)

        print("RC after arm frames:", read_rc(sock))
        print("Status after arm frames:", read_status_ex(sock))

        print("Small throttle test...")
        send_for(10.0, sock, armed_small_throttle, rate_hz=50)

        print("Back to throttle low...")
        send_for(0.5, sock, armed_low_throttle, rate_hz=50)

        print("DISARM...")
        send_for(1.0, sock, disarmed, rate_hz=50)

        print("Final status:", read_status_ex(sock))


if __name__ == "__main__":
    main()