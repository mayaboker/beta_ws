import socket
import struct
from typing import Tuple, Dict, Any

MSP_API_VERSION   = 1
MSP_STATUS        = 101
MSP_RAW_IMU       = 102
MSP_RC            = 105
MSP_ATTITUDE      = 108
MSP_ALTITUDE      = 109
MSP_ANALOG        = 110
MSP_BATTERY_STATE = 130
MSP_STATUS_EX     = 150


def msp_v1_packet(cmd: int, payload: bytes = b"") -> bytes:
    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b
    return b"$M<" + bytes([size, cmd]) + payload + bytes([checksum])


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed while reading MSP reply")
        buf += chunk
    return buf


def read_msp_frame(sock: socket.socket) -> Tuple[int, bytes]:
    # MSP v1 response header is $M>
    header = recv_exact(sock, 3)
    if header != b"$M>":
        raise ValueError(f"Unexpected MSP header: {header!r}")

    size = recv_exact(sock, 1)[0]
    cmd = recv_exact(sock, 1)[0]
    payload = recv_exact(sock, size)
    checksum = recv_exact(sock, 1)[0]

    calc = size ^ cmd
    for b in payload:
        calc ^= b

    if checksum != calc:
        raise ValueError(
            f"Bad MSP checksum: got {checksum:#x}, expected {calc:#x}"
        )

    return cmd, payload


def msp_request(sock: socket.socket, cmd: int, payload: bytes = b"") -> bytes:
    sock.sendall(msp_v1_packet(cmd, payload))
    resp_cmd, resp_payload = read_msp_frame(sock)
    if resp_cmd != cmd:
        raise ValueError(f"Reply command mismatch: got {resp_cmd}, expected {cmd}")
    return resp_payload


def parse_api_version(payload: bytes) -> Dict[str, Any]:
    # Common MSP_API_VERSION reply is 3 bytes: protocol, major, minor
    if len(payload) < 3:
        raise ValueError("MSP_API_VERSION payload too short")
    protocol, major, minor = struct.unpack("<BBB", payload[:3])
    return {
        "protocol_version": protocol,
        "api_major": major,
        "api_minor": minor,
    }


def parse_status(payload: bytes) -> Dict[str, Any]:
    # MSP_STATUS comment says:
    # cycletime, errors_count, sensor present, box activation, current setting number
    # Common legacy layout: <HHHI B
    if len(payload) < 11:
        raise ValueError("MSP_STATUS payload too short")
    cycle_time, i2c_errors, sensors, mode_flags, profile = struct.unpack("<HHHIB", payload[:11])
    return {
        "cycle_time_us": cycle_time,
        "i2c_errors": i2c_errors,
        "sensors_bitmask": sensors,
        "mode_flags": mode_flags,
        "profile_index": profile,
    }


def parse_status_ex(payload: bytes) -> Dict[str, Any]:
    # Betaflight documents STATUS_EX as an extended status reply
    # Layouts can differ by API version, so keep parsing conservative.
    if len(payload) < 13:
        raise ValueError("MSP_STATUS_EX payload too short")
    cycle_time, i2c_errors, sensors, mode_flags, profile, cpu_load = struct.unpack("<HHHIBH", payload[:13])
    return {
        "cycle_time_us": cycle_time,
        "i2c_errors": i2c_errors,
        "sensors_bitmask": sensors,
        "mode_flags": mode_flags,
        "profile_index": profile,
        "cpu_load_tenths_percent_or_fw_specific": cpu_load,
        "raw_payload_hex": payload.hex(),  # keep the rest visible for API-specific decode
    }


def parse_raw_imu(payload: bytes) -> Dict[str, Any]:
    # 9 signed 16-bit values: acc xyz, gyro xyz, mag xyz
    if len(payload) < 18:
        raise ValueError("MSP_RAW_IMU payload too short")
    vals = struct.unpack("<9h", payload[:18])
    return {
        "acc": vals[0:3],
        "gyro": vals[3:6],
        "mag": vals[6:9],
    }


def parse_rc(payload: bytes) -> Dict[str, Any]:
    # Sequence of uint16 channel values
    if len(payload) % 2 != 0:
        raise ValueError("MSP_RC payload length must be even")
    channels = struct.unpack("<" + "H" * (len(payload) // 2), payload)
    return {"channels": channels}


def parse_attitude(payload: bytes) -> Dict[str, Any]:
    # 2 angles + 1 heading
    if len(payload) < 6:
        raise ValueError("MSP_ATTITUDE payload too short")
    ang_x, ang_y, heading = struct.unpack("<3h", payload[:6])
    return {
        "roll_tenths_deg": ang_x,
        "pitch_tenths_deg": ang_y,
        "heading_deg": heading,
    }


def parse_altitude(payload: bytes) -> Dict[str, Any]:
    # Altitude + variometer
    if len(payload) < 6:
        raise ValueError("MSP_ALTITUDE payload too short")
    altitude_cm, vario_cms = struct.unpack("<ih", payload[:6])
    return {
        "altitude_cm": altitude_cm,
        "vario_cm_s": vario_cms,
    }


def parse_analog(payload: bytes) -> Dict[str, Any]:
    # Comment says vbat, powermetersum, rssi
    # Common legacy layout: <B H H
    if len(payload) < 5:
        raise ValueError("MSP_ANALOG payload too short")
    vbat_0p1v, power_meter_sum, rssi = struct.unpack("<BHH", payload[:5])
    return {
        "battery_volts": vbat_0p1v / 10.0,
        "power_meter_sum": power_meter_sum,
        "rssi": rssi,
    }


def parse_battery_state(payload: bytes) -> Dict[str, Any]:
    # Betaflight defines this as connected/disconnected, voltage, current used
    # Exact layout can vary by API version, so decode minimally and keep raw.
    result = {"raw_payload_hex": payload.hex()}
    if len(payload) >= 1:
        result["cell_count_or_state_byte0"] = payload[0]
    if len(payload) >= 3:
        result["voltage_raw_le16"] = struct.unpack("<H", payload[1:3])[0]
    if len(payload) >= 5:
        result["mah_drawn"] = struct.unpack("<H", payload[3:5])[0]
    if len(payload) >= 7:
        result["amperage_raw"] = struct.unpack("<H", payload[5:7])[0]
    return result


def main():
    host = "127.0.0.1"
    port = 5761  # SITL UART1 default

    with socket.create_connection((host, port), timeout=2.0) as sock:
        sock.settimeout(1.0)

        api = parse_api_version(msp_request(sock, MSP_API_VERSION))
        print("API:", api)

        print("STATUS_EX:", parse_status_ex(msp_request(sock, MSP_STATUS_EX)))
        print("RC:", parse_rc(msp_request(sock, MSP_RC)))
        print("ATTITUDE:", parse_attitude(msp_request(sock, MSP_ATTITUDE)))
        print("ALTITUDE:", parse_altitude(msp_request(sock, MSP_ALTITUDE)))
        print("ANALOG:", parse_analog(msp_request(sock, MSP_ANALOG)))
        print("RAW_IMU:", parse_raw_imu(msp_request(sock, MSP_RAW_IMU)))
        print("BATTERY_STATE:", parse_battery_state(msp_request(sock, MSP_BATTERY_STATE)))


if __name__ == "__main__":
    main()