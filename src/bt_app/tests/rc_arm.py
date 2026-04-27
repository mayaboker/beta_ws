import struct
import time
import serial

MSP_API_VERSION = 1
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_SET_RAW_RC = 200

PORT = "link=/tmp/ttyBF"     # change to /dev/ttyAMA0, COM3, etc.
BAUD = 115200             # common MSP baud rate


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


def read_msp_v1(ser: serial.Serial, timeout: float = 0.5):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue

        if b != b"$":
            continue

        if ser.read(1) != b"M":
            continue

        direction = ser.read(1)
        if direction not in (b">", b"!"):
            continue

        size_b = ser.read(1)
        cmd_b = ser.read(1)
        if not size_b or not cmd_b:
            continue

        size = size_b[0]
        cmd = cmd_b[0]
        payload = ser.read(size)
        checksum_b = ser.read(1)

        if len(payload) != size or not checksum_b:
            raise TimeoutError("Incomplete MSP frame")

        checksum = size ^ cmd
        for x in payload:
            checksum ^= x

        if checksum != checksum_b[0]:
            raise ValueError("Bad MSP checksum")

        if direction == b"!":
            raise RuntimeError(f"MSP error response for command {cmd}")

        return cmd, payload

    raise TimeoutError("No MSP response")


def request_msp(ser: serial.Serial, cmd: int, timeout: float = 0.5) -> bytes:
    ser.write(msp_v1_frame(cmd))
    while True:
        rcmd, payload = read_msp_v1(ser, timeout=timeout)
        if rcmd == cmd:
            return payload


def send_raw_rc(ser: serial.Serial, channels):
    """
    channels: 8 RC values, normally 1000..2000 microseconds.
    Default Betaflight AETR1234:
      CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw,
      CH5 AUX1, CH6 AUX2, CH7 AUX3, CH8 AUX4
    """
    if len(channels) != 8:
        raise ValueError("MSP_SET_RAW_RC example uses exactly 8 channels")

    payload = struct.pack("<8H", *channels)
    ser.write(msp_v1_frame(MSP_SET_RAW_RC, payload))

    # Betaflight may return an empty ACK. Try to consume it so the serial
    # buffer does not fill, but do not block the RC stream.
    old_timeout = ser.timeout
    ser.timeout = 0.005
    try:
        try:
            read_msp_v1(ser, timeout=0.005)
        except TimeoutError:
            pass
    finally:
        ser.timeout = old_timeout


def read_attitude(ser: serial.Serial):
    payload = request_msp(ser, MSP_ATTITUDE)
    roll_x10, pitch_x10, heading = struct.unpack_from("<hhh", payload)
    return {
        "roll_deg": roll_x10 / 10.0,
        "pitch_deg": pitch_x10 / 10.0,
        "heading_deg": heading,
    }


def read_altitude(ser: serial.Serial):
    payload = request_msp(ser, MSP_ALTITUDE)
    # Common MSP_ALTITUDE layout:
    # int32 altitude_cm, int16 vario_cm_s
    altitude_cm, vario_cm_s = struct.unpack_from("<ih", payload)
    return {
        "altitude_m": altitude_cm / 100.0,
        "vertical_speed_m_s": vario_cm_s / 100.0,
    }


def read_raw_imu(ser: serial.Serial):
    payload = request_msp(ser, MSP_RAW_IMU)
    values = struct.unpack_from("<9h", payload)
    return {
        "acc": values[0:3],
        "gyro": values[3:6],
        "mag": values[6:9],
    }


def send_for(duration_s: float, ser: serial.Serial, channels, rate_hz: float = 50):
    dt = 1.0 / rate_hz
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        send_raw_rc(ser, channels)
        time.sleep(dt)


def main():
    # Default AETR1234 example:
    # CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw, CH5 AUX1/ARM
    disarmed = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
    armed_low_throttle = [1500, 1500, 1000, 1500, 1800, 1000, 1000, 1000]

    # Very low motor test only. This is not hover throttle.
    armed_small_throttle = [1500, 1500, 1050, 1500, 1800, 1000, 1000, 1000]

    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        time.sleep(1.0)
        ser.reset_input_buffer()

        # Check communication.
        api = request_msp(ser, MSP_API_VERSION)
        print("MSP API raw:", api.hex(" "))

        print("Attitude:", read_attitude(ser))
        print("Altitude:", read_altitude(ser))
        print("Raw IMU:", read_raw_imu(ser))

        # Safety: props off for this test.
        print("Sending disarmed neutral...")
        send_for(1.0, ser, disarmed)

        print("ARM: AUX1 high, throttle low...")
        send_for(1.0, ser, armed_low_throttle)

        print("Small throttle test...")
        send_for(1.0, ser, armed_small_throttle)

        print("Back to throttle low...")
        send_for(0.5, ser, armed_low_throttle)

        print("DISARM...")
        send_for(1.0, ser, disarmed)


if __name__ == "__main__":
    main()