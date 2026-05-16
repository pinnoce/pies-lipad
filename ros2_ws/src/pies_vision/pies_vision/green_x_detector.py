#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np
import cv2

# ── Tunable parameters ────────────────────────────────────────────────────────
# Green HSV range — tweak S/V bounds at the field if lighting changes
GREEN_LOW  = (35,  60,  60)   # H, S, V lower bound
GREEN_HIGH = (85, 255, 255)   # H, S, V upper bound

# Ignore detections smaller than this (filters grass/noise at altitude)
MIN_AREA_PX = 300

# Ignore detections larger than this fraction of total frame area (filters
# cases where the whole background is green, e.g. grassy field)
MAX_AREA_FRAC = 0.30

# ─────────────────────────────────────────────────────────────────────────────

class GreenXDetector(Node):
    def __init__(self):
        super().__init__('green_x_detector')
        self.create_subscription(Image, '/camera/image_raw', self._cb, 1)
        self.pub = self.create_publisher(Point, '/vision/green_x', 10)
        self.pub_debug = self.create_publisher(Image, '/vision/green_x/debug', 1)
        self.get_logger().info('Green X detector started')

    def _cb(self, msg):
        # Decode NV21 → BGR
        yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height * 3 // 2, msg.width)
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)

        # Green mask
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)

        # Remove speckle, fill gaps
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        # Find largest green blob
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pt = Point()  # x/y = pixel offset from frame centre, z = area (0 = not detected)
        cx, cy = msg.width // 2, msg.height // 2

        max_area = msg.width * msg.height * MAX_AREA_FRAC
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if MIN_AREA_PX <= area <= max_area:
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    px = int(M['m10'] / M['m00'])
                    py = int(M['m01'] / M['m00'])
                    pt.x = float(px - cx)
                    pt.y = float(py - cy)
                    pt.z = float(area)
                    self.get_logger().info(
                        f'Green X: offset=({pt.x:+.0f}, {pt.y:+.0f}) px  area={area:.0f} px²'
                    )

        self.pub.publish(pt)
        self._publish_debug(msg, bgr, mask, pt, cx, cy)

    def _publish_debug(self, msg, bgr, mask, pt, cx, cy):
        if self.pub_debug.get_subscription_count() == 0:
            return
        debug = bgr.copy()
        # Overlay green mask in bright green
        debug[mask > 0] = (0, 255, 0)
        # Crosshair at frame centre
        cv2.line(debug, (cx, 0), (cx, msg.height), (200, 200, 200), 1)
        cv2.line(debug, (0, cy), (msg.width, cy), (200, 200, 200), 1)
        # Mark detection
        if pt.z > 0:
            det_x, det_y = int(pt.x + cx), int(pt.y + cy)
            cv2.circle(debug, (det_x, det_y), 12, (0, 0, 255), -1)
            cv2.line(debug, (cx, cy), (det_x, det_y), (0, 0, 255), 2)

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = 'bgr8'
        out.step = msg.width * 3
        out.data = debug.tobytes()
        self.pub_debug.publish(out)


def main():
    rclpy.init()
    node = GreenXDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
