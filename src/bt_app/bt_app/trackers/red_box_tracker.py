#!/usr/bin/env python3

import argparse
from dataclasses import asdict
import json
import math
import threading
from dataclasses import dataclass

import cv2
import numpy as np
import zmq
from loguru import logger

from bt_app.common import (
    CAMERA_HORIZONTAL_FOV_RAD,
    DEFAULT_RED_BOX_TRACKER_WINDOW,
    GAZEBO_CAMERA_TOPIC,
    ZMQ_CAMERA_ENDPOINT,
    ZMQ_CAMERA_TOPIC,
    ZMQ_TRACKER_RESULT_ENDPOINT,
    ZMQ_TRACKER_RESULT_TOPIC,
)
from bt_app.msgs import TrackerResult
from bt_app.sensors.gz_camera import GazeboCameraPublisher
from bt_app.trackers.image_display import image_to_bgr


@dataclass(frozen=True)
class RedBoxDetection:
    center_px: tuple[int, int]
    bbox: tuple[int, int, int, int]
    area_px: float
    error_px: tuple[float, float]
    tracker_result: TrackerResult
    error_angle_rad: tuple[float, float]
    error_angle_deg: tuple[float, float]


class RedBoxTracker:
    """Track a red box in camera frames received over ZMQ."""

    def __init__(
        self,
        *,
        zmq_endpoint=ZMQ_CAMERA_ENDPOINT,
        zmq_topic=ZMQ_CAMERA_TOPIC,
        result_endpoint=ZMQ_TRACKER_RESULT_ENDPOINT,
        result_topic=ZMQ_TRACKER_RESULT_TOPIC,
        horizontal_fov_rad=CAMERA_HORIZONTAL_FOV_RAD,
        min_area_px=150.0,
        display=True,
        window_name=DEFAULT_RED_BOX_TRACKER_WINDOW,
        context=None,
    ):
        self.zmq_endpoint = zmq_endpoint
        self.zmq_topic = zmq_topic
        self.result_endpoint = result_endpoint
        self.result_topic = result_topic
        self.horizontal_fov_rad = horizontal_fov_rad
        self.min_area_px = min_area_px
        self.display = display
        self.window_name = window_name
        self.context = context or zmq.Context.instance()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.RCVHWM, 2)
        self.subscriber.setsockopt(zmq.SUBSCRIBE, self.zmq_topic)
        self.result_publisher = None

    def start(self):
        self.result_publisher = self.context.socket(zmq.PUB)
        self.result_publisher.setsockopt(zmq.SNDHWM, 2)
        self.result_publisher.bind(self.result_endpoint)
        self.subscriber.connect(self.zmq_endpoint)
        logger.info(
            "Red box tracker subscribed to {} on topic {}",
            self.zmq_endpoint,
            self.zmq_topic.decode("utf-8", errors="replace"),
        )
        logger.info(
            "Red box tracker publishing results to {} on topic {}",
            self.result_endpoint,
            self.result_topic.decode("utf-8", errors="replace"),
        )

    def close(self):
        self.subscriber.close(linger=0)
        if self.result_publisher is not None:
            self.result_publisher.close(linger=0)
            self.result_publisher = None
        if self.display:
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
                    detection = detect_red_box(
                        frame,
                        horizontal_fov_rad=self.horizontal_fov_rad,
                        min_area_px=self.min_area_px,
                    )
                    self.handle_detection(frame, detection)

                if self.display:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        logger.info("Stopping red box tracker")
                        break
        finally:
            self.close()

    def handle_detection(self, frame, detection):
        if detection is None:
            logger.debug("No red box detected")
            if self.display:
                draw_crosshair(frame)
                cv2.imshow(self.window_name, frame)
            return

        self.publish_tracker_result(detection.tracker_result)

        logger.info(
            "red_box center_px={} error_px=({:.1f}, {:.1f}) "
            "error_angle_deg=({:.2f}, {:.2f}) area_px={:.1f}",
            detection.center_px,
            detection.error_px[0],
            detection.error_px[1],
            math.degrees(detection.tracker_result.error_x),
            math.degrees(detection.tracker_result.error_y),
            detection.area_px,
        )

        if self.display:
            draw_detection(frame, detection)
            cv2.imshow(self.window_name, frame)

    def publish_tracker_result(self, tracker_result):
        if self.result_publisher is None:
            raise RuntimeError("RedBoxTracker.start() must be called before publish")

        payload = json.dumps(asdict(tracker_result)).encode("utf-8")
        self.result_publisher.send_multipart([self.result_topic, payload])
        logger.debug("Published tracker result {}", tracker_result)


def detect_red_box(frame, *, horizontal_fov_rad, min_area_px):
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red_1 = np.array([0, 80, 60], dtype=np.uint8)
    upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red_2 = np.array([170, 80, 60], dtype=np.uint8)
    upper_red_2 = np.array([179, 255, 255], dtype=np.uint8)

    mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.bitwise_or(mask_1, mask_2)

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _hierarchy = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area_px:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    center_x = x + w // 2
    center_y = y + h // 2
    error_x_px = center_x - (width / 2.0)
    error_y_px = (height / 2.0) - center_y

    vertical_fov_rad = vertical_fov_from_horizontal(
        horizontal_fov_rad,
        width,
        height,
    )
    error_x_rad = (error_x_px / (width / 2.0)) * (horizontal_fov_rad / 2.0)
    error_y_rad = (error_y_px / (height / 2.0)) * (vertical_fov_rad / 2.0)

    return RedBoxDetection(
        center_px=(center_x, center_y),
        bbox=(x, y, w, h),
        area_px=area,
        error_px=(error_x_px, error_y_px),
        tracker_result=TrackerResult(
            error_x=error_x_rad,
            error_y=error_y_rad,
        ),
        error_angle_rad=(error_x_rad, error_y_rad),
        error_angle_deg=(math.degrees(error_x_rad), math.degrees(error_y_rad)),
    )


def vertical_fov_from_horizontal(horizontal_fov_rad, width, height):
    aspect_height_over_width = height / width
    return 2.0 * math.atan(
        math.tan(horizontal_fov_rad / 2.0) * aspect_height_over_width
    )


def draw_detection(frame, detection):
    draw_crosshair(frame)
    x, y, w, h = detection.bbox
    cx, cy = detection.center_px
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)
    cv2.putText(
        frame,
        f"err deg x={detection.error_angle_deg[0]:.2f} y={detection.error_angle_deg[1]:.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def draw_crosshair(frame):
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    cv2.drawMarker(
        frame,
        center,
        (255, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=24,
        thickness=1,
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Track a red box in the Gazebo camera stream."
    )
    parser.add_argument("--zmq-endpoint", default=ZMQ_CAMERA_ENDPOINT)
    parser.add_argument("--result-endpoint", default=ZMQ_TRACKER_RESULT_ENDPOINT)
    parser.add_argument("--gazebo-topic", default=GAZEBO_CAMERA_TOPIC)
    parser.add_argument("--window", default=DEFAULT_RED_BOX_TRACKER_WINDOW)
    parser.add_argument("--horizontal-fov", type=float, default=CAMERA_HORIZONTAL_FOV_RAD)
    parser.add_argument("--min-area", type=float, default=150.0)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--no-camera-publisher",
        action="store_true",
        help="Only subscribe to ZMQ. Use this when another publisher is already running.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    context = zmq.Context.instance()
    publisher_stop = None
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

    tracker = RedBoxTracker(
        zmq_endpoint=args.zmq_endpoint,
        result_endpoint=args.result_endpoint,
        horizontal_fov_rad=args.horizontal_fov,
        min_area_px=args.min_area,
        display=not args.no_display,
        window_name=args.window,
        context=context,
    )

    try:
        tracker.spin()
    finally:
        if publisher_stop is not None:
            publisher_stop.set()
        if publisher_thread is not None:
            publisher_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
