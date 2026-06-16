#!/usr/bin/env python3
"""
yolo_classifier — subscribes to /camera/color/image_raw, runs YOLOv8 object
detection, then classifies each detected region by HSV colour.

Publishes the dominant colour class ('red' | 'green' | 'blue' | 'unknown')
to /object_class at sensor rate.

Dependencies:
  pip install ultralytics opencv-python-headless
  sudo apt install ros-$ROS_DISTRO-cv-bridge

If ultralytics is not installed the node falls back to full-frame HSV
colour segmentation (works well with the brightly coloured cubes in
object.sdf).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

import numpy as np
import cv2
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False


# HSV ranges for colour classification (OpenCV: H in [0,179])
_COLOUR_RANGES = {
    'red':   [((0,   100,  80), (10,  255, 255)),
              ((165, 100,  80), (179, 255, 255))],
    'green': [((35,   80,  60), (85,  255, 255))],
    'blue':  [((95,   80,  60), (135, 255, 255))],
}


def _dominant_colour(bgr_roi: np.ndarray) -> str:
    """Return the colour name with the most pixels in the ROI."""
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    best_colour, best_count = 'unknown', 0
    for colour, ranges in _COLOUR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_colour = colour
    # Require at least 1 % of pixels to be coloured to avoid noise hits
    threshold = int(0.01 * bgr_roi.shape[0] * bgr_roi.shape[1])
    return best_colour if best_count >= threshold else 'unknown'


class YoloClassifier(Node):
    def __init__(self):
        super().__init__('yolo_classifier')

        self._bridge = CvBridge()
        self._pub = self.create_publisher(String, '/object_class', 10)
        self._sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._image_cb, 10)

        if _YOLO_AVAILABLE:
            # yolov8n.pt (~6 MB) is auto-downloaded on first run
            self._model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8n loaded — using detection + colour')
        else:
            self._model = None
            self.get_logger().warn(
                'ultralytics not installed — using HSV colour fallback. '
                'Install with: pip install ultralytics')

    # ── helpers ───────────────────────────────────────────────────────

    def _classify_frame(self, bgr: np.ndarray) -> str:
        """Classify the central 60 % of the frame by colour."""
        h, w = bgr.shape[:2]
        roi = bgr[h // 5: 4 * h // 5, w // 5: 4 * w // 5]
        return _dominant_colour(roi)

    def _classify_with_yolo(self, bgr: np.ndarray) -> str:
        results = self._model(bgr, verbose=False, conf=0.25)
        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            return self._classify_frame(bgr)

        # Take the highest-confidence detected box
        boxes = results[0].boxes
        best = int(boxes.conf.argmax())
        x1, y1, x2, y2 = map(int, boxes.xyxy[best].tolist())
        # Guard against degenerate boxes
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(bgr.shape[1], x2), min(bgr.shape[0], y2)
        roi = bgr[y1:y2, x1:x2]
        if roi.size == 0:
            return self._classify_frame(bgr)
        return _dominant_colour(roi)

    # ── callback ──────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        colour = (self._classify_with_yolo(bgr)
                  if self._model is not None
                  else self._classify_frame(bgr))

        out = String()
        out.data = colour
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloClassifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
