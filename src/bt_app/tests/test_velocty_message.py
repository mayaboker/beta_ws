#!/usr/bin/env python3

from __future__ import annotations

import json
import signal
import time
from dataclasses import asdict
from threading import Event

import zmq

from bt_app.common import ZMQ_LINEAR_VELOCITY_TOPIC, ZMQ_ODOMETRY_ENDPOINT
from bt_app.msgs import LinearVelocity


def main() -> None:
    stop_event = Event()

    def stop_handler(signum: int, _frame: object) -> None:
        print(f"Stopping velocity test publisher after signal {signum}")
        stop_event.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    context = zmq.Context.instance()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 10)
    publisher.bind(ZMQ_ODOMETRY_ENDPOINT)

    samples = [
        LinearVelocity(x=0.0, y=0.0, z=0.0),
        LinearVelocity(x=0.5, y=0.0, z=0.0),
        LinearVelocity(x=1.0, y=0.2, z=0.0),
        LinearVelocity(x=0.2, y=-0.4, z=0.1),
    ]

    index = 0
    print(
        f"Publishing test velocity messages on {ZMQ_ODOMETRY_ENDPOINT} "
        f"topic {ZMQ_LINEAR_VELOCITY_TOPIC.decode('utf-8')}"
    )

    try:
        # Give subscribers a moment to connect before the first PUB message.
        time.sleep(0.2)
        while not stop_event.is_set():
            velocity = samples[index % len(samples)]
            publisher.send_multipart(
                [
                    ZMQ_LINEAR_VELOCITY_TOPIC,
                    json.dumps(asdict(velocity)).encode("utf-8"),
                ]
            )
            print(f"Sent velocity: {velocity}")
            index += 1
            time.sleep(1.0)
    finally:
        publisher.close(linger=0)


if __name__ == "__main__":
    main()
