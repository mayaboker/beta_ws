#!/usr/bin/env python3

import argparse
import json
import time
from dataclasses import dataclass

import zmq
from loguru import logger

from bt_app.common import ZMQ_TRACKER_RESULT_ENDPOINT, ZMQ_TRACKER_RESULT_TOPIC
from bt_app.msp import (
    DEFAULT_SITL_HOST,
    DEFAULT_SITL_PORT,
    BetaflightMspClient,
)
from bt_app.msgs import TrackerResult


RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000
ARM_LOW = 1000
ARM_HIGH = 1900


@dataclass(frozen=True)
class MissionConfig:
    target_altitude_m: float = 5.0
    hold_duration_s: float = 20.0
    hover_throttle: int = 1450
    min_throttle: int = 1100
    max_throttle: int = 1750
    kp: float = 220.0
    kd: float = 90.0
    ki: float = 25.0
    integral_limit: float = 1.0
    rate_hz: float = 50.0
    landing_duration_s: float = 5.0


class TrackerResultSubscriber:
    def __init__(
        self,
        *,
        endpoint=ZMQ_TRACKER_RESULT_ENDPOINT,
        topic=ZMQ_TRACKER_RESULT_TOPIC,
        context=None,
    ):
        self.endpoint = endpoint
        self.topic = topic
        self.context = context or zmq.Context.instance()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.RCVHWM, 10)
        self.subscriber.setsockopt(zmq.SUBSCRIBE, self.topic)
        self.poller = zmq.Poller()
        self.latest = None

    def start(self):
        self.subscriber.connect(self.endpoint)
        self.poller.register(self.subscriber, zmq.POLLIN)
        logger.info(
            "Listening for tracker results on {} topic {}",
            self.endpoint,
            self.topic.decode("utf-8", errors="replace"),
        )

    def close(self):
        self.poller.unregister(self.subscriber)
        self.subscriber.close(linger=0)

    def poll_latest(self, timeout_ms=0):
        while True:
            events = dict(self.poller.poll(timeout_ms))
            timeout_ms = 0
            if self.subscriber not in events:
                return self.latest

            _topic, payload = self.subscriber.recv_multipart()
            data = json.loads(payload.decode("utf-8"))
            error_x = float(data["error_x"])
            error_y=float(data["error_y"])
            self.latest = TrackerResult(
                error_x,
                error_y,
            )

            logger.info(f"tracker x,y ({error_x}{error_y})")


class AltitudeHoverController:
    def __init__(self, config: MissionConfig):
        self.config = config
        self.integral_error = 0.0

    def throttle_for(self, altitude_m: float, vertical_speed_m_s: float, dt: float):
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


def takeoff_and_hold(msp, tracker_sub, config, *, log_every_s=0.5):
    controller = AltitudeHoverController(config)
    dt = 1.0 / config.rate_hz
    end = time.monotonic() + config.hold_duration_s
    next_log = time.monotonic()
    last_throttle = config.hover_throttle

    logger.info(
        "Takeoff and hold: target_altitude={:.2f}m duration={:.1f}s",
        config.target_altitude_m,
        config.hold_duration_s,
    )

    while time.monotonic() < end:
        loop_start = time.monotonic()
        altitude = msp.read_altitude()
        tracker_result = tracker_sub.poll_latest()
        throttle = controller.throttle_for(
            altitude["altitude_m"],
            altitude["vertical_speed_m_s"],
            dt,
        )
        last_throttle = throttle

        msp.send_raw_rc(make_channels(throttle, armed=True))

        if loop_start >= next_log:
            attitude = msp.read_attitude()
            logger.info(
                "mission alt={:.2f}m vz={:.2f}m/s throttle={} "
                "roll={:.1f} pitch={:.1f} tracker={}",
                altitude["altitude_m"],
                altitude["vertical_speed_m_s"],
                throttle,
                attitude["roll_deg"],
                attitude["pitch_deg"],
                tracker_result,
            )
            next_log = loop_start + log_every_s

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, dt - elapsed))

    return last_throttle


def land(msp, *, from_throttle: int, duration_s: float, rate_hz: float):
    steps = max(1, int(duration_s * rate_hz))
    dt = 1.0 / rate_hz

    logger.info("Landing: ramp throttle {} -> {}", from_throttle, RC_MIN)
    for i in range(steps):
        t = i / max(1, steps - 1)
        throttle = int(from_throttle + (RC_MIN - from_throttle) * t)
        msp.send_raw_rc(make_channels(throttle, armed=True))
        time.sleep(dt)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Take off to 5 m, listen to tracker results, then land."
    )
    parser.add_argument("--host", default=DEFAULT_SITL_HOST)
    parser.add_argument("--port", default=DEFAULT_SITL_PORT, type=int)
    parser.add_argument("--tracker-endpoint", default=ZMQ_TRACKER_RESULT_ENDPOINT)
    parser.add_argument("--target-altitude", default=5.0, type=float)
    parser.add_argument("--hold-duration", default=20.0, type=float)
    parser.add_argument("--landing-duration", default=5.0, type=float)
    parser.add_argument("--hover-throttle", default=1450, type=int)
    parser.add_argument("--min-throttle", default=1100, type=int)
    parser.add_argument("--max-throttle", default=1750, type=int)
    parser.add_argument("--kp", default=220.0, type=float)
    parser.add_argument("--kd", default=90.0, type=float)
    parser.add_argument("--ki", default=25.0, type=float)
    parser.add_argument("--rate-hz", default=50.0, type=float)
    return parser.parse_args()


def main():
    args = parse_args()
    config = MissionConfig(
        target_altitude_m=args.target_altitude,
        hold_duration_s=args.hold_duration,
        hover_throttle=args.hover_throttle,
        min_throttle=args.min_throttle,
        max_throttle=args.max_throttle,
        kp=args.kp,
        kd=args.kd,
        ki=args.ki,
        rate_hz=args.rate_hz,
        landing_duration_s=args.landing_duration,
    )
    disarmed = make_channels(RC_MIN, armed=False)
    armed_low_throttle = make_channels(RC_MIN, armed=True)

    tracker_sub = TrackerResultSubscriber(endpoint=args.tracker_endpoint)
    tracker_sub.start()

    try:
        with BetaflightMspClient(args.host, args.port) as msp:
            time.sleep(2.0)
            logger.info("MSP API raw: {}", msp.read_api_version_raw().hex(" "))
            logger.info("Initial RC: {}", msp.read_rc())
            logger.info("Initial status: {}", msp.read_status_ex())

            logger.info("Waiting for CALIBRATING to clear...")
            status = msp.wait_until_not_calibrating(timeout_s=15.0)
            logger.info("Calibration cleared: {}", status)

            logger.info("Sending disarmed neutral...")
            msp.send_for(2.0, disarmed, rate_hz=config.rate_hz)

            logger.info("ARM: AUX1 high, throttle low...")
            msp.send_for(3.0, armed_low_throttle, rate_hz=config.rate_hz)

            last_throttle = config.hover_throttle
            try:
                last_throttle = takeoff_and_hold(msp, tracker_sub, config)
            finally:
                land(
                    msp,
                    from_throttle=last_throttle,
                    duration_s=config.landing_duration_s,
                    rate_hz=config.rate_hz,
                )
                logger.info("DISARM...")
                msp.send_for(1.0, disarmed, rate_hz=config.rate_hz)
                logger.info("Final status: {}", msp.read_status_ex())
    finally:
        tracker_sub.close()


if __name__ == "__main__":
    main()
