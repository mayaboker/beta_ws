#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import signal
from threading import Event

import zmq

from bt_app.common import ZMQ_LINEAR_VELOCITY_TOPIC, ZMQ_ODOMETRY_ENDPOINT
from bt_app.msgs import LinearVelocity


class ZmqOdometryViewer:
    def __init__(
        self,
        *,
        zmq_endpoint: str = ZMQ_ODOMETRY_ENDPOINT,
        zmq_topic: bytes = ZMQ_LINEAR_VELOCITY_TOPIC,
        context: zmq.Context | None = None,
    ) -> None:
        self.zmq_endpoint = zmq_endpoint
        self.zmq_topic = zmq_topic
        self.context = context or zmq.Context.instance()
        self.subscriber: zmq.Socket | None = None

    def start(self) -> None:
        if self.subscriber is not None:
            return

        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.RCVHWM, 10)
        self.subscriber.setsockopt(zmq.SUBSCRIBE, self.zmq_topic)
        self.subscriber.connect(self.zmq_endpoint)

    def close(self) -> None:
        if self.subscriber is not None:
            self.subscriber.close(linger=0)
            self.subscriber = None

    def read(self, timeout_ms: int = 1000) -> LinearVelocity | None:
        if self.subscriber is None:
            raise RuntimeError("ZmqOdometryViewer.start() must be called before read")

        poller = zmq.Poller()
        poller.register(self.subscriber, zmq.POLLIN)
        events = dict(poller.poll(timeout_ms))
        if self.subscriber not in events:
            return None

        _topic, payload = self.subscriber.recv_multipart()
        data = json.loads(payload.decode("utf-8"))
        return LinearVelocity(
            x=float(data["x"]),
            y=float(data["y"]),
            z=float(data["z"]),
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Listen to odometry linear velocity over ZMQ and print it to the console."
    )
    parser.add_argument("--zmq-endpoint", default=ZMQ_ODOMETRY_ENDPOINT)
    parser.add_argument(
        "--topic",
        default=ZMQ_LINEAR_VELOCITY_TOPIC.decode("utf-8"),
        help="ZMQ topic prefix to subscribe to.",
    )
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=1000,
        help="Poll timeout in milliseconds before printing a waiting message.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    stop_event = Event()

    def stop_handler(signum: int, _frame: object) -> None:
        print(f"Stopping odometry viewer after signal {signum}")
        stop_event.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    viewer = ZmqOdometryViewer(
        zmq_endpoint=args.zmq_endpoint,
        zmq_topic=args.topic.encode("utf-8"),
    )
    viewer.start()

    print(f"Listening on {args.zmq_endpoint} topic {args.topic}")

    try:
        while not stop_event.is_set():
            velocity = viewer.read(timeout_ms=args.timeout_ms)
            if velocity is None:
                print("Waiting for odometry message...")
                continue

            print(
                f"linear velocity: x={velocity.x:.3f}, y={velocity.y:.3f}, z={velocity.z:.3f}"
            )
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
