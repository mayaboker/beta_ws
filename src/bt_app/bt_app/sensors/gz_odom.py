#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import queue
import signal
from dataclasses import asdict
from threading import Event

import gz.transport13 as gz_transport
import zmq
from gz.msgs10 import odometry_pb2
from loguru import logger

from bt_app.common import (
    GAZEBO_ODOMETRY_TOPIC,
    ZMQ_LINEAR_VELOCITY_TOPIC,
    ZMQ_ODOMETRY_ENDPOINT,
)
from bt_app.msgs import LinearVelocity


class GazeboOdometryPublisher:
    """Bridge Gazebo odometry linear velocity to a ZMQ PUB socket."""

    def __init__(
        self,
        *,
        gazebo_topic: str = GAZEBO_ODOMETRY_TOPIC,
        zmq_endpoint: str = ZMQ_ODOMETRY_ENDPOINT,
        zmq_topic: bytes = ZMQ_LINEAR_VELOCITY_TOPIC,
        context: zmq.Context | None = None,
    ) -> None:
        self.gazebo_topic: str = gazebo_topic
        self.zmq_endpoint: str = zmq_endpoint
        self.zmq_topic: bytes = zmq_topic
        self.context: zmq.Context = context or zmq.Context.instance()
        self.node = gz_transport.Node()
        self.publisher: zmq.Socket | None = None
        self.velocities: queue.Queue[LinearVelocity] = queue.Queue(maxsize=10)
        self.started: bool = False

    def start(self) -> None:
        if self.started:
            return

        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.setsockopt(zmq.SNDHWM, 10)
        self.publisher.bind(self.zmq_endpoint)
        self.node.subscribe(odometry_pb2.Odometry, self.gazebo_topic, self._on_odometry)
        self.started = True
        logger.info(
            "Publishing Gazebo odometry linear velocity {} to ZMQ {} on topic {}",
            self.gazebo_topic,
            self.zmq_endpoint,
            self.zmq_topic.decode("utf-8", errors="replace"),
        )

    def close(self) -> None:
        if self.publisher is not None:
            self.publisher.close(linger=0)
            self.publisher = None
        self.started = False

    def spin(
        self,
        poll_interval_s: float = 0.01,
        install_signal_handlers: bool = True,
        stop_event: Event | None = None,
    ) -> None:
        stop = False

        def stop_handler(signum: int, _frame: object) -> None:
            nonlocal stop
            logger.info("Stopping odometry publisher after signal {}", signum)
            stop = True

        if install_signal_handlers:
            signal.signal(signal.SIGINT, stop_handler)
            signal.signal(signal.SIGTERM, stop_handler)

        self.start()
        try:
            while not stop and not (stop_event is not None and stop_event.is_set()):
                self.publish_pending(timeout_s=poll_interval_s)
        finally:
            self.close()

    def publish_pending(self, timeout_s: float = 0.0) -> None:
        if self.publisher is None:
            raise RuntimeError("GazeboOdometryPublisher.start() must be called before publish")

        try:
            velocity = self.velocities.get(timeout=timeout_s)
        except queue.Empty:
            return

        self._publish_velocity(velocity)

        while True:
            try:
                velocity = self.velocities.get_nowait()
            except queue.Empty:
                break

            self._publish_velocity(velocity)

    def _publish_velocity(self, velocity: LinearVelocity) -> None:
        if self.publisher is None:
            raise RuntimeError("GazeboOdometryPublisher.start() must be called before publish")

        print(f"Publishing velocity: {velocity}")
        self.publisher.send_multipart(
            [
                self.zmq_topic,
                json.dumps(asdict(velocity)).encode("utf-8"),
            ]
        )

    def _on_odometry(self, msg: odometry_pb2.Odometry) -> None:
        velocity = LinearVelocity(
            x=float(msg.twist.linear.x),
            y=float(msg.twist.linear.y),
            z=float(msg.twist.linear.z),
        )

        if self.velocities.full():
            try:
                self.velocities.get_nowait()
            except queue.Empty:
                pass

        self.velocities.put_nowait(velocity)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish Gazebo Harmonic odometry linear velocity to a ZMQ PUB socket."
    )
    parser.add_argument("--gazebo-topic", default=GAZEBO_ODOMETRY_TOPIC)
    parser.add_argument("--zmq-endpoint", default=ZMQ_ODOMETRY_ENDPOINT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    publisher = GazeboOdometryPublisher(
        gazebo_topic=args.gazebo_topic,
        zmq_endpoint=args.zmq_endpoint,
    )
    publisher.spin()


if __name__ == "__main__":
    main()
