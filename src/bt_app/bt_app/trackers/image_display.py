#!/usr/bin/env python3

import argparse
import json
import threading

import cv2
import numpy as np
import zmq
from gz.msgs10 import image_pb2
from loguru import logger

from bt_app.common import (
    DEFAULT_CAMERA_WINDOW,
    GAZEBO_CAMERA_TOPIC,
    ZMQ_CAMERA_ENDPOINT,
    ZMQ_CAMERA_TOPIC,
)
from bt_app.sensors.gz_camera import GazeboCameraPublisher


PIXEL_FORMAT_NAMES = {
    value.number: value.name
    for value in image_pb2.PixelFormatType.DESCRIPTOR.values
}


class ImageDisplayTracker:
    """Subscribe to in-process ZMQ camera frames and display them with OpenCV."""

    def __init__(
        self,
        *,
        zmq_endpoint=ZMQ_CAMERA_ENDPOINT,
        zmq_topic=ZMQ_CAMERA_TOPIC,
        window_name=DEFAULT_CAMERA_WINDOW,
        context=None,
    ):
        self.zmq_endpoint = zmq_endpoint
        self.zmq_topic = zmq_topic
        self.window_name = window_name
        self.context = context or zmq.Context.instance()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.RCVHWM, 2)
        self.subscriber.setsockopt(zmq.SUBSCRIBE, self.zmq_topic)

    def start(self):
        self.subscriber.connect(self.zmq_endpoint)
        logger.info(
            "Subscribed to ZMQ camera {} on topic {}",
            self.zmq_endpoint,
            self.zmq_topic.decode("utf-8", errors="replace"),
        )

    def close(self):
        self.subscriber.close(linger=0)
        cv2.destroyAllWindows()

    def spin(self, timeout_ms=100):
        self.start()
        poller = zmq.Poller()
        poller.register(self.subscriber, zmq.POLLIN)

        try:
            while True:
                events = dict(poller.poll(timeout_ms))
                if self.subscriber in events:
                    _topic, metadata_bytes, image_bytes = self.subscriber.recv_multipart()
                    metadata = json.loads(metadata_bytes.decode("utf-8"))
                    frame = image_to_bgr(metadata, image_bytes)
                    cv2.imshow(self.window_name, frame)
                    logger.debug("Displayed camera frame {}", metadata.get("frame"))

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    logger.info("Stopping image display tracker")
                    break
        finally:
            self.close()


def image_to_bgr(metadata, data):
    width = metadata["width"]
    height = metadata["height"]
    step = metadata["step"]
    pixel_format = metadata["pixel_format_type"]

    if width <= 0 or height <= 0:
        raise ValueError(f"Invalid image size: {width}x{height}")

    if pixel_format == image_pb2.RGB_INT8:
        image = image_buffer(data, height, width, step, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if pixel_format == image_pb2.BGR_INT8:
        return image_buffer(data, height, width, step, 3)

    if pixel_format == image_pb2.RGBA_INT8:
        image = image_buffer(data, height, width, step, 4)
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    if pixel_format == image_pb2.BGRA_INT8:
        image = image_buffer(data, height, width, step, 4)
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    if pixel_format == image_pb2.L_INT8:
        gray = image_buffer(data, height, width, step, 1).reshape((height, width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    name = PIXEL_FORMAT_NAMES.get(pixel_format, f"UNKNOWN({pixel_format})")
    raise ValueError(f"Unsupported Gazebo image pixel format: {name}")


def image_buffer(data, height, width, step, channels):
    row_bytes = width * channels
    step = step or row_bytes

    if step < row_bytes:
        raise ValueError(
            f"Image step {step} is smaller than row size {row_bytes}"
        )

    expected_size = step * height
    if len(data) < expected_size:
        raise ValueError(
            f"Image data too short: got {len(data)} bytes, need {expected_size}"
        )

    image = np.frombuffer(data, dtype=np.uint8, count=expected_size)
    image = image.reshape((height, step))[:, :row_bytes]
    return image.reshape((height, width, channels))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Display inproc ZMQ camera images with OpenCV."
    )
    parser.add_argument("--zmq-endpoint", default=ZMQ_CAMERA_ENDPOINT)
    parser.add_argument("--gazebo-topic", default=GAZEBO_CAMERA_TOPIC)
    parser.add_argument("--window", default=DEFAULT_CAMERA_WINDOW)
    parser.add_argument(
        "--no-camera-publisher",
        action="store_true",
        help="Only subscribe to ZMQ. Use this when another publisher is running in the same process.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    context = zmq.Context.instance()
    publisher = None
    publisher_thread = None

    if not args.no_camera_publisher:
        publisher_stop = threading.Event()
        publisher = GazeboCameraPublisher(
            gazebo_topic=args.gazebo_topic,
            zmq_endpoint=args.zmq_endpoint,
            context=context,
        )
        publisher_thread = threading.Thread(
            target=publisher.spin,
            kwargs={
                "install_signal_handlers": False,
                "stop_event": publisher_stop,
            },
            daemon=True,
        )
        publisher_thread.start()

    tracker = ImageDisplayTracker(
        zmq_endpoint=args.zmq_endpoint,
        window_name=args.window,
        context=context,
    )
    try:
        tracker.spin()
    finally:
        if publisher is not None:
            publisher_stop.set()
        if publisher_thread is not None:
            publisher_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
