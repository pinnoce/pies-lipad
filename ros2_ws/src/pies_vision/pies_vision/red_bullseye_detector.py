#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np
import cv2

# ── Tunable parameters ────────────────────────────────────────────────────────
# Red wraps in HSV — need two ranges that together cover all reds
RED_LOW1,  RED_HIGH1  = (  0, 80, 80), ( 10, 255, 255)  # lower red band
RED_LOW2,  RED_HIGH2  = (170, 80, 80), (180, 255, 255)  # upper red band

# Minimum blob area (px²) — filters small red objects and random red patches.
# At competition altitude (10–30 m), the 5 m bullseye spans 50–200 px radius
# (area ≥ ~7 800 px²). 2000 rejects isolated red speckles while still finding
# a distant target; raise it further if false positives remain at the field.
MIN_AREA_PX = 2000

# Circularity threshold (0–1, 1 = perfect circle)
# Bullseye is a circle so this filters out non-circular red patches
MIN_CIRCULARITY = 0.5
# ─────────────────────────────────────────────────────────────────────────────


class RedBullseyeDetector(Node):
    def __init__(self):
        super().__init__('red_bullseye_detector')
        self.create_subscription(Image, '/camera/image_raw', self._cb, 1)
        self.pub = self.create_publisher(Point, '/vision/red_bullseye', 10)
        self.pub_debug = self.create_publisher(Image, '/vision/red_bullseye/debug', 1)
        self.get_logger().info('Red bullseye detector started')

    def _cb(self, msg):
        # Decode NV21 → BGR
        yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height * 3 // 2, msg.width)
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)

        # Red mask (two HSV bands combined)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LOW1, RED_HIGH1),
            cv2.inRange(hsv, RED_LOW2, RED_HIGH2),
        )

        # Remove speckle, fill gaps
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pt = Point()  # x/y = pixel offset from frame centre, z = area (0 = not detected)
        cx, cy = msg.width // 2, msg.height // 2
        best_circ = 0.0

        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA_PX:
                continue
            # Circularity check — bullseye is round, random red patches are not
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity < MIN_CIRCULARITY:
                continue
            # Keep the most circular blob (not largest): circularity discriminates
            # real bullseyes (0.8+) from background rust/noise blobs (0.5–0.6)
            if circularity > best_circ:
                best_circ = circularity
                M = cv2.moments(c)
                if M['m00'] > 0:
                    px = int(M['m10'] / M['m00'])
                    py = int(M['m01'] / M['m00'])
                    pt.x = float(px - cx)
                    pt.y = float(py - cy)
                    pt.z = float(area)

        if pt.z > 0:
            self.get_logger().info(
                f'Bullseye: offset=({pt.x:+.0f}, {pt.y:+.0f}) px  area={pt.z:.0f} px²'
            )

        self.pub.publish(pt)
        self._publish_debug(msg, bgr, mask, pt, cx, cy)

    def _publish_debug(self, msg, bgr, mask, pt, cx, cy):
        if self.pub_debug.get_subscription_count() == 0:
            return
        debug = bgr.copy()
        debug[mask > 0] = (0, 0, 255)
        cv2.line(debug, (cx, 0), (cx, msg.height), (200, 200, 200), 1)
        cv2.line(debug, (0, cy), (msg.width, cy), (200, 200, 200), 1)
        if pt.z > 0:
            det_x, det_y = int(pt.x + cx), int(pt.y + cy)
            cv2.circle(debug, (det_x, det_y), 15, (0, 255, 255), 3)
            cv2.line(debug, (cx, cy), (det_x, det_y), (0, 255, 255), 2)

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
    node = RedBullseyeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
