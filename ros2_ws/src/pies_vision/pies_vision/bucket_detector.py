#!/usr/bin/env python3
"""
Detects the competition bucket by colour (blue, orange, or green).

Finds the largest saturated blob matching any of the three known bucket
colours and reports its centroid offset from the frame centre.

Topics:
  sub  /camera/image_raw       sensor_msgs/Image
  pub  /vision/bucket          std_msgs/String   (JSON; empty '{}' when not found)
  pub  /vision/bucket/debug    sensor_msgs/Image

JSON output:
  {"color": "orange", "offset_x": 12, "offset_y": -5, "area": 4800}

Parameter:
  target_color  "any" (default) | "blue" | "orange" | "green"
                Set via --ros-args -p target_color:=orange to filter one colour.
"""
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2

# ── Tunable parameters ────────────────────────────────────────────────────────
# HSV ranges chosen from the actual bucket colours (OV5647 camera, outdoor light).
# Orange V_min=150 excludes Mojave rust terrain (rust V≈90–155, bucket V≈180+).
COLOURS = {
    'orange': (np.array([  8, 150, 150]), np.array([ 22, 255, 255])),
    'green':  (np.array([ 38, 100,  60]), np.array([ 82, 255, 255])),
    'blue':   (np.array([ 95, 100,  60]), np.array([130, 255, 255])),
}

# Minimum blob area (px²). At 5 m altitude over a 25 cm bucket ≈ 2 500 px²;
# 800 px² allows detection from ~8 m and filters noise/small colour patches.
MIN_AREA_PX = 800

# Max fraction of frame area a single blob may occupy (full-frame masks = FP).
MAX_AREA_FRAC = 0.35
# ─────────────────────────────────────────────────────────────────────────────


class BucketDetector(Node):
    def __init__(self):
        super().__init__('bucket_detector')

        self.declare_parameter('target_color', 'any')

        self.create_subscription(Image, '/camera/image_raw', self._cb, 1)
        self.pub       = self.create_publisher(String, '/vision/bucket', 10)
        self.pub_debug = self.create_publisher(Image, '/vision/bucket/debug', 1)

        target = self.get_parameter('target_color').value
        if target != 'any' and target not in COLOURS:
            self.get_logger().error(
                f'Unknown target_color "{target}" — must be any/blue/orange/green'
            )
        self.get_logger().info(f'Bucket detector started  target={target}')

    def _cb(self, msg):
        yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height * 3 // 2, msg.width)
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        target   = self.get_parameter('target_color').value
        max_area = msg.width * msg.height * MAX_AREA_FRAC
        cx, cy   = msg.width // 2, msg.height // 2
        k        = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        best = None   # (area, colour_name, px, py, contour)

        candidates = {target: COLOURS[target]} if target != 'any' else COLOURS

        for colour_name, (lo, hi) in candidates.items():
            mask = cv2.inRange(hsv, lo, hi)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)
            if not (MIN_AREA_PX <= area <= max_area):
                continue

            M = cv2.moments(largest)
            if M['m00'] == 0:
                continue
            px = int(M['m10'] / M['m00'])
            py = int(M['m01'] / M['m00'])

            if best is None or area > best[0]:
                best = (area, colour_name, px, py, largest)

        debug = bgr.copy()
        cv2.line(debug, (cx, 0),        (cx, msg.height), (200, 200, 200), 1)
        cv2.line(debug, (0,  cy),       (msg.width, cy),  (200, 200, 200), 1)

        if best:
            area, colour_name, px, py, contour = best
            det = {
                'color':    colour_name,
                'offset_x': px - cx,
                'offset_y': py - cy,
                'area':     int(area),
            }
            self.pub.publish(String(data=json.dumps(det)))
            self.get_logger().info(
                f'Bucket [{colour_name}]: offset=({det["offset_x"]:+d},{det["offset_y"]:+d}) '
                f'area={area:.0f}'
            )

            # Debug overlay
            colour_bgr = {'orange': (0, 165, 255), 'green': (0, 220, 0), 'blue': (255, 100, 0)}
            col = colour_bgr.get(colour_name, (255, 255, 0))
            cv2.drawContours(debug, [contour], -1, col, 2)
            cv2.circle(debug, (px, py), 10, col, -1)
            cv2.line(debug, (cx, cy), (px, py), col, 2)
            cv2.putText(debug, colour_name, (px + 12, py), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        else:
            self.pub.publish(String(data='{}'))

        self._publish_debug(msg, debug)

    def _publish_debug(self, msg, debug):
        if self.pub_debug.get_subscription_count() == 0:
            return
        out = Image()
        out.header   = msg.header
        out.height   = msg.height
        out.width    = msg.width
        out.encoding = 'bgr8'
        out.step     = msg.width * 3
        out.data     = debug.tobytes()
        self.pub_debug.publish(out)


def main():
    rclpy.init()
    node = BucketDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
