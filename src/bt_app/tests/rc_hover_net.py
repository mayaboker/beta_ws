#!/usr/bin/env python3

import argparse
import socket
import time
from dataclasses import dataclass

from rc_arm_net import (
    MSP_API_VERSION,
    SITL_HOST,
    SITL_PORT,
    read_altitude,
    read_attitude,
    read_rc,
    read_status_ex,
    request_msp,
    send_for,
    send_raw_rc,
    wait_until_not_calibrating,
)


RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000
ARM_LOW = 1000
ARM_HIGH = 1900


@dataclass
class HoverConfig:
    target_altitude_m: float = 1.0
    hover_throttle: int = 1450
    min_throttle: int = 1100
    max_throttle: int = 1700
    kp: float = 220.0
    kd: float = 90.0
    ki: float = 25.0
    integral_limit: float = 1.0
    rate_hz: float = 50.0
    duration_s: float = 20.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def make_channels(
    throttle: int,
    *,
    armed: bool,
    roll: int = RC_MID,
    pitch: int = RC_MID,
    yaw: int = RC_MID,
):
    return [
        roll,
        pitch,
        int(clamp(throttle, RC_MIN, RC_MAX)),
        yaw,
        ARM_HIGH if armed else ARM_LOW,
        RC_MIN,
        RC_MIN,
        RC_MIN,
    ]


class AltitudeHoverController:
    def __init__(self, config: HoverConfig):
        self.config = config
        self.integral_error = 0.0

    def throttle_for(self, altitude_m: float, vertical_speed_m_s: float, dt: float) -> int:
        error = self.config.target_altitude_m - altitude_m
        self.integral_error = clamp(
            self.integral_error + error * dt,
            -self.config.integral_limit,
            self.config.integral_limit,
        )

        throttle = (
            self.config.hover_throttle
            + self.config.kp * error
            - self.config.kd * vertical_speed_m_s
            + self.config.ki * self.integral_error
        )

        return int(clamp(throttle, self.config.min_throttle, self.config.max_throttle))


def send_hover(
    sock: socket.socket,
    config: HoverConfig,
    *,
    print_every_s: float = 0.5,
):
    controller = AltitudeHoverController(config)
    dt = 1.0 / config.rate_hz
    last_throttle = config.hover_throttle
    next_print = time.monotonic()
    end = time.monotonic() + config.duration_s

    while time.monotonic() < end:
        loop_start = time.monotonic()
        altitude = read_altitude(sock)
        throttle = controller.throttle_for(
            altitude["altitude_m"],
            altitude["vertical_speed_m_s"],
            dt,
        )
        last_throttle = throttle

        send_raw_rc(sock, make_channels(throttle, armed=True))

        if loop_start >= next_print:
            attitude = read_attitude(sock)
            print(
                "hover "
                f"alt={altitude['altitude_m']:.2f}m "
                f"vz={altitude['vertical_speed_m_s']:.2f}m/s "
                f"thr={throttle} "
                f"roll={attitude['roll_deg']:.1f} "
                f"pitch={attitude['pitch_deg']:.1f}"
            )
            next_print = loop_start + print_every_s

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, dt - elapsed))

    return last_throttle


def land(
    sock: socket.socket,
    *,
    from_throttle: int,
    duration_s: float = 4.0,
    rate_hz: float = 50.0,
):
    steps = max(1, int(duration_s * rate_hz))
    dt = 1.0 / rate_hz

    for i in range(steps):
        t = i / max(1, steps - 1)
        throttle = int(from_throttle + (RC_MIN - from_throttle) * t)
        send_raw_rc(sock, make_channels(throttle, armed=True))
        time.sleep(dt)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Arm Betaflight SITL over MSP and hold altitude with throttle feedback."
    )
    parser.add_argument("--host", default=SITL_HOST)
    parser.add_argument("--port", default=SITL_PORT, type=int)
    parser.add_argument("--target-altitude", default=1.0, type=float)
    parser.add_argument("--duration", default=20.0, type=float)
    parser.add_argument("--hover-throttle", default=1450, type=int)
    parser.add_argument("--min-throttle", default=1100, type=int)
    parser.add_argument("--max-throttle", default=1700, type=int)
    parser.add_argument("--kp", default=220.0, type=float)
    parser.add_argument("--kd", default=90.0, type=float)
    parser.add_argument("--ki", default=25.0, type=float)
    parser.add_argument("--rate-hz", default=50.0, type=float)
    return parser.parse_args()


def main():
    args = parse_args()
    config = HoverConfig(
        target_altitude_m=args.target_altitude,
        hover_throttle=args.hover_throttle,
        min_throttle=args.min_throttle,
        max_throttle=args.max_throttle,
        kp=args.kp,
        kd=args.kd,
        ki=args.ki,
        rate_hz=args.rate_hz,
        duration_s=args.duration,
    )

    disarmed = make_channels(RC_MIN, armed=False)
    armed_low_throttle = make_channels(RC_MIN, armed=True)

    with socket.create_connection((args.host, args.port), timeout=2.0) as sock:
        sock.settimeout(0.05)
        print(f"Connected to Betaflight SITL MSP at {args.host}:{args.port}")

        time.sleep(2.0)
        api = request_msp(sock, MSP_API_VERSION)
        print("MSP API raw:", api.hex(" "))
        print("Initial RC:", read_rc(sock))
        print("Initial status:", read_status_ex(sock))

        print("Waiting for CALIBRATING to clear...")
        status = wait_until_not_calibrating(sock, timeout_s=15.0)
        print("Calibration cleared:", status)

        print("Sending disarmed neutral...")
        send_for(2.0, sock, disarmed, rate_hz=config.rate_hz)

        print("ARM: AUX1 high, throttle low...")
        send_for(3.0, sock, armed_low_throttle, rate_hz=config.rate_hz)

        print(
            "Hover: "
            f"target={config.target_altitude_m:.2f}m "
            f"duration={config.duration_s:.1f}s "
            f"base_throttle={config.hover_throttle}"
        )
        last_throttle = send_hover(sock, config)

        print("Landing...")
        land(sock, from_throttle=last_throttle, rate_hz=config.rate_hz)

        print("DISARM...")
        send_for(1.0, sock, disarmed, rate_hz=config.rate_hz)
        print("Final status:", read_status_ex(sock))


if __name__ == "__main__":
    main()
