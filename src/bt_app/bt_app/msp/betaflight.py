#!/usr/bin/env python3

import socket
import struct
import time

from loguru import logger


MSP_API_VERSION = 1
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_STATUS_EX = 150
MSP_SET_RAW_RC = 200

DEFAULT_SITL_HOST = "127.0.0.1"
DEFAULT_SITL_PORT = 5761


ARMING_DISABLE_FLAGS = {
    0: "NO_GYRO",
    1: "FAILSAFE",
    2: "RX_FAILSAFE",
    3: "NOT_DISARMED",
    4: "BOXFAILSAFE",
    5: "RUNAWAY_TAKEOFF",
    6: "CRASH_DETECTED",
    7: "THROTTLE",
    8: "ANGLE",
    9: "BOOT_GRACE_TIME",
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


def msp_v1_frame(cmd: int, payload: bytes = b"") -> bytes:
    if not 0 <= cmd <= 255:
        raise ValueError("MSP v1 command must be 0..255")
    if len(payload) > 255:
        raise ValueError("MSP v1 payload too large")

    size = len(payload)
    checksum = size ^ cmd
    for byte in payload:
        checksum ^= byte

    return b"$M<" + bytes([size, cmd]) + payload + bytes([checksum])


def decode_arming_mask(mask: int):
    return [
        name
        for bit, name in ARMING_DISABLE_FLAGS.items()
        if mask & (1 << bit)
    ]


class BetaflightMspClient:
    """TCP MSP v1 client for Betaflight SITL."""

    def __init__(
        self,
        host=DEFAULT_SITL_HOST,
        port=DEFAULT_SITL_PORT,
        *,
        connect_timeout_s=2.0,
        socket_timeout_s=0.05,
    ):
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.socket_timeout_s = socket_timeout_s
        self.sock = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.close()

    def connect(self):
        if self.sock is not None:
            return

        self.sock = socket.create_connection(
            (self.host, self.port),
            timeout=self.connect_timeout_s,
        )
        self.sock.settimeout(self.socket_timeout_s)
        logger.info("Connected to Betaflight SITL MSP at {}:{}", self.host, self.port)

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _require_socket(self):
        if self.sock is None:
            raise RuntimeError("BetaflightMspClient is not connected")
        return self.sock

    def recv_exact(self, n: int, timeout: float = 0.5) -> bytes:
        sock = self._require_socket()
        sock.settimeout(timeout)

        data = b""
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
            except socket.timeout:
                raise TimeoutError(f"Timeout while waiting for {n} bytes") from None

            if not chunk:
                raise ConnectionError("Socket closed by Betaflight SITL")

            data += chunk

        return data

    def read_msp_v1(self, timeout: float = 0.5):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())

            try:
                byte = self.recv_exact(1, timeout=remaining)
            except TimeoutError:
                continue

            if byte != b"$":
                continue

            remaining = max(0.001, deadline - time.monotonic())

            try:
                if self.recv_exact(1, timeout=remaining) != b"M":
                    continue

                direction = self.recv_exact(1, timeout=remaining)
                if direction not in (b">", b"!"):
                    continue

                size = self.recv_exact(1, timeout=remaining)[0]
                cmd = self.recv_exact(1, timeout=remaining)[0]
                payload = self.recv_exact(size, timeout=remaining)
                received_checksum = self.recv_exact(1, timeout=remaining)[0]

            except TimeoutError:
                raise TimeoutError("Incomplete MSP frame") from None

            checksum = size ^ cmd
            for byte in payload:
                checksum ^= byte

            if checksum != received_checksum:
                raise ValueError(
                    f"Bad MSP checksum: got 0x{received_checksum:02x}, "
                    f"expected 0x{checksum:02x}"
                )

            if direction == b"!":
                raise RuntimeError(f"MSP error response for command {cmd}")

            return cmd, payload

        raise TimeoutError("No MSP response")

    def request_msp(self, cmd: int, timeout: float = 0.5) -> bytes:
        sock = self._require_socket()
        sock.sendall(msp_v1_frame(cmd))

        while True:
            response_cmd, payload = self.read_msp_v1(timeout=timeout)
            if response_cmd == cmd:
                return payload

    def send_raw_rc(self, channels):
        """
        Send 8 RC values, normally 1000..2000 microseconds.

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
            raise ValueError("MSP_SET_RAW_RC uses exactly 8 channels")

        sock = self._require_socket()
        payload = struct.pack("<8H", *channels)
        sock.sendall(msp_v1_frame(MSP_SET_RAW_RC, payload))

        old_timeout = sock.gettimeout()
        sock.settimeout(0.005)

        try:
            try:
                self.read_msp_v1(timeout=0.005)
            except TimeoutError:
                pass
        finally:
            sock.settimeout(old_timeout)

    def send_for(self, duration_s: float, channels, rate_hz: float = 50):
        dt = 1.0 / rate_hz
        end = time.monotonic() + duration_s

        while time.monotonic() < end:
            self.send_raw_rc(channels)
            time.sleep(dt)

    def read_api_version_raw(self) -> bytes:
        return self.request_msp(MSP_API_VERSION)

    def read_rc(self):
        payload = self.request_msp(MSP_RC)
        count = len(payload) // 2
        return list(struct.unpack_from("<" + "H" * count, payload))

    def read_attitude(self):
        payload = self.request_msp(MSP_ATTITUDE)
        roll_x10, pitch_x10, heading = struct.unpack_from("<hhh", payload)

        return {
            "roll_deg": roll_x10 / 10.0,
            "pitch_deg": pitch_x10 / 10.0,
            "heading_deg": heading,
        }

    def read_altitude(self):
        payload = self.request_msp(MSP_ALTITUDE)
        altitude_cm, vario_cm_s = struct.unpack_from("<ih", payload)

        return {
            "altitude_m": altitude_cm / 100.0,
            "vertical_speed_m_s": vario_cm_s / 100.0,
        }

    def read_raw_imu(self):
        payload = self.request_msp(MSP_RAW_IMU)
        values = struct.unpack_from("<9h", payload)

        return {
            "acc": values[0:3],
            "gyro": values[3:6],
            "mag": values[6:9],
        }

    def read_status_ex(self):
        payload = self.request_msp(MSP_STATUS_EX)

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

    def wait_until_not_calibrating(self, timeout_s=15.0):
        end = time.monotonic() + timeout_s

        while time.monotonic() < end:
            status = self.read_status_ex()
            flags = status["arming_disable_flags"]

            logger.info("Waiting for calibration: {}", flags)

            if "CALIBRATING" not in flags:
                return status

            self.send_raw_rc([1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000])
            time.sleep(0.1)

        raise TimeoutError("CALIBRATING did not clear")

