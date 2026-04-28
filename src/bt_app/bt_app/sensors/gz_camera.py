#!/usr/bin/env python3

import argparse
import json
import queue
import signal

import gz.transport13 as gz_transport
import zmq
from gz.msgs10 import image_pb2
from loguru import logger

from bt_app.common import (
    GAZEBO_CAMERA_TOPIC,
    ZMQ_CAMERA_ENDPOINT,
    ZMQ_CAMERA_TOPIC,
)


class GazeboCameraPublisher:
    """Bridge Gazebo Harmonic image messages to an in-process ZMQ PUB socket."""

    def __init__(
        self,
        *,
        gazebo_topic=GAZEBO_CAMERA_TOPIC,
        zmq_endpoint=ZMQ_CAMERA_ENDPOINT,
        zmq_topic=ZMQ_CAMERA_TOPIC,
        context=None,
    ):
        self.gazebo_topic = gazebo_topic
        self.zmq_endpoint = zmq_endpoint
        self.zmq_topic = zmq_topic
        self.context = context or zmq.Context.instance()
        self.node = gz_transport.Node()
        self.publisher = None
        self.frames = queue.Queue(maxsize=2)
        self.frame_count = 0
        self.started = False

    def start(self):
        if self.started:
            return

        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.setsockopt(zmq.SNDHWM, 2)
        self.publisher.bind(self.zmq_endpoint)
        self.node.subscribe(image_pb2.Image, self.gazebo_topic, self._on_image)
        self.started = True
        logger.info(
            "Publishing Gazebo camera {} to ZMQ {} on topic {}",
            self.gazebo_topic,
            self.zmq_endpoint,
            self.zmq_topic.decode("utf-8", errors="replace"),
        )

    def close(self):
        if self.publisher is not None:
            self.publisher.close(linger=0)
            self.publisher = None
        self.started = False

    def spin(
        self,
        poll_interval_s=0.01,
        install_signal_handlers=True,
        stop_event=None,
    ):
        stop = False

        def stop_handler(signum, _frame):
            nonlocal stop
            logger.info("Stopping camera publisher after signal {}", signum)
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

    def publish_pending(self, timeout_s=0.0):
        if self.publisher is None:
            raise RuntimeError("GazeboCameraPublisher.start() must be called before publish")

        try:
            metadata, data = self.frames.get(timeout=timeout_s)
        except queue.Empty:
            return

        self.publisher.send_multipart(
            [
                self.zmq_topic,
                json.dumps(metadata).encode("utf-8"),
                data,
            ]
        )

        while True:
            try:
                metadata, data = self.frames.get_nowait()
            except queue.Empty:
                break

            self.publisher.send_multipart(
                [
                    self.zmq_topic,
                    json.dumps(metadata).encode("utf-8"),
                    data,
                ]
            )

    def _on_image(self, msg):
        metadata = {
            "width": msg.width,
            "height": msg.height,
            "step": msg.step,
            "pixel_format_type": msg.pixel_format_type,
            "frame": self.frame_count,
        }
        self.frame_count += 1

        if self.frames.full():
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass

        self.frames.put_nowait((metadata, bytes(msg.data)))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish Gazebo Harmonic camera images to an inproc ZMQ PUB socket."
    )
    parser.add_argument("--gazebo-topic", default=GAZEBO_CAMERA_TOPIC)
    parser.add_argument("--zmq-endpoint", default=ZMQ_CAMERA_ENDPOINT)
    return parser.parse_args()


def main():
    args = parse_args()
    publisher = GazeboCameraPublisher(
        gazebo_topic=args.gazebo_topic,
        zmq_endpoint=args.zmq_endpoint,
    )
    publisher.spin()


if __name__ == "__main__":
    main()
