#!/usr/bin/env python3

import time

from loguru import logger

from bt_app.msp import (
    DEFAULT_SITL_HOST,
    DEFAULT_SITL_PORT,
    BetaflightMspClient,
)


SITL_HOST = DEFAULT_SITL_HOST
SITL_PORT = DEFAULT_SITL_PORT


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

    with BetaflightMspClient(SITL_HOST, SITL_PORT) as msp:
        # Wait for SITL boot/calibration.
        time.sleep(2.0)

        api = msp.read_api_version_raw()
        logger.info("MSP API raw: {}", api.hex(" "))

        logger.info("Initial status: {}", msp.read_status_ex())
        logger.info("Initial RC: {}", msp.read_rc())

        logger.info("Attitude: {}", msp.read_attitude())
        logger.info("Altitude: {}", msp.read_altitude())
        logger.info("Raw IMU: {}", msp.read_raw_imu())

        logger.info("Waiting for CALIBRATING to clear...")
        status = msp.wait_until_not_calibrating(timeout_s=15.0)
        logger.info("Calibration cleared: {}", status)

        logger.info("Sending disarmed neutral...")
        msp.send_for(2.0, disarmed, rate_hz=50)

        logger.info("RC after disarmed frames: {}", msp.read_rc())
        logger.info("Status after disarmed frames: {}", msp.read_status_ex())

        logger.info("ARM: AUX1 high, throttle low...")
        msp.send_for(20.0, armed_low_throttle, rate_hz=50)

        logger.info("RC after arm frames: {}", msp.read_rc())
        logger.info("Status after arm frames: {}", msp.read_status_ex())

        logger.info("Small throttle test...")
        msp.send_for(10.0, armed_small_throttle, rate_hz=50)

        logger.info("Back to throttle low...")
        msp.send_for(0.5, armed_low_throttle, rate_hz=50)

        logger.info("DISARM...")
        msp.send_for(1.0, disarmed, rate_hz=50)

        logger.info("Final status: {}", msp.read_status_ex())


if __name__ == "__main__":
    main()
