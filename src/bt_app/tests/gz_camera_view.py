#!/usr/bin/env python3

import argparse
import queue
import sys
import time

import cv2
import gz.transport13 as gz_transport
import numpy as np
from gz.msgs10 import image_pb2


DEFAULT_TOPIC = "/camera"


PIXEL_FORMAT_NAMES = {
    value.number: value.name
    for value in image_pb2.PixelFormatType.DESCRIPTOR.values
}


def image_msg_to_bgr(msg):
    width = msg.width
    height = msg.height
    step = msg.step
    pixel_format = msg.pixel_format_type
    data = msg.data

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
        description="Display a Gazebo Harmonic camera topic using gz.transport13 and OpenCV."
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"Gazebo image topic to subscribe to. Default: {DEFAULT_TOPIC}",
    )
    parser.add_argument(
        "--window",
        default="Gazebo Camera",
        help="OpenCV window title.",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for the first image before printing a warning.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    frames = queue.Queue(maxsize=1)
    last_error = {"message": None}

    def on_image(msg):
        try:
            frame = image_msg_to_bgr(msg)
        except ValueError as exc:
            last_error["message"] = str(exc)
            return

        if frames.full():
            try:
                frames.get_nowait()
            except queue.Empty:
                pass
        frames.put_nowait(frame)

    node = gz_transport.Node()
    node.subscribe(image_pb2.Image, args.topic, on_image)

    print(f"Subscribed to Gazebo camera topic: {args.topic}")
    print("Press q or Escape in the image window to exit.")

    first_frame_deadline = time.monotonic() + args.wait_timeout
    warned_waiting = False

    while True:
        try:
            frame = frames.get(timeout=0.05)
        except queue.Empty:
            if (
                not warned_waiting
                and args.wait_timeout > 0
                and time.monotonic() > first_frame_deadline
            ):
                if last_error["message"]:
                    print(last_error["message"], file=sys.stderr)
                else:
                    print(f"Waiting for images on {args.topic}...", file=sys.stderr)
                warned_waiting = True

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            continue

        cv2.imshow(args.window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
