#!/usr/bin/env python3

import argparse
import time
from dataclasses import dataclass

from loguru import logger

from bt_app.msp import (
    DEFAULT_SITL_HOST,
    DEFAULT_SITL_PORT,
    BetaflightMspClient,
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
    msp: BetaflightMspClient,
    config: HoverConfig,
    *,
    log_every_s: float = 0.5,
):
    controller = AltitudeHoverController(config)
    dt = 1.0 / config.rate_hz
    last_throttle = config.hover_throttle
    next_print = time.monotonic()
    end = time.monotonic() + config.duration_s

    while time.monotonic() < end:
        loop_start = time.monotonic()
        altitude = msp.read_altitude()
        throttle = controller.throttle_for(
            altitude["altitude_m"],
            altitude["vertical_speed_m_s"],
            dt,
        )
        last_throttle = throttle

        msp.send_raw_rc(make_channels(throttle, armed=True))

        if loop_start >= next_print:
            attitude = msp.read_attitude()
            logger.info(
                "hover "
                "alt={:.2f}m "
                "vz={:.2f}m/s "
                "thr={} "
                "roll={:.1f} "
                "pitch={:.1f}",
                altitude["altitude_m"],
                altitude["vertical_speed_m_s"],
                throttle,
                attitude["roll_deg"],
                attitude["pitch_deg"],
            )
            next_print = loop_start + log_every_s

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, dt - elapsed))

    return last_throttle


def land(
    msp: BetaflightMspClient,
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
        msp.send_raw_rc(make_channels(throttle, armed=True))
        time.sleep(dt)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Arm Betaflight SITL over MSP and hold altitude with throttle feedback."
    )
    parser.add_argument("--host", default=DEFAULT_SITL_HOST)
    parser.add_argument("--port", default=DEFAULT_SITL_PORT, type=int)
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

    with BetaflightMspClient(args.host, args.port) as msp:
        time.sleep(2.0)
        api = msp.read_api_version_raw()
        logger.info("MSP API raw: {}", api.hex(" "))
        logger.info("Initial RC: {}", msp.read_rc())
        logger.info("Initial status: {}", msp.read_status_ex())

        logger.info("Waiting for CALIBRATING to clear...")
        status = msp.wait_until_not_calibrating(timeout_s=15.0)
        logger.info("Calibration cleared: {}", status)

        logger.info("Sending disarmed neutral...")
        msp.send_for(2.0, disarmed, rate_hz=config.rate_hz)

        logger.info("ARM: AUX1 high, throttle low...")
        msp.send_for(3.0, armed_low_throttle, rate_hz=config.rate_hz)

        logger.info(
            "Hover: target={:.2f}m duration={:.1f}s base_throttle={}",
            config.target_altitude_m,
            config.duration_s,
            config.hover_throttle,
        )
        last_throttle = send_hover(msp, config)

        logger.info("Landing...")
        land(msp, from_throttle=last_throttle, rate_hz=config.rate_hz)

        logger.info("DISARM...")
        msp.send_for(1.0, disarmed, rate_hz=config.rate_hz)
        logger.info("Final status: {}", msp.read_status_ex())


if __name__ == "__main__":
    main()
