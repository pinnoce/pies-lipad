#!/usr/bin/env python3
"""
Detects the black/white triangle markers used in Target Localization.

Large marker (1.2 m): 4-triangle square, no number — localization only.
Small marker (0.6 m): same pattern + number in the right white triangle.

Topics:
  sub  /camera/image_raw        sensor_msgs/Image
  pub  /vision/markers          std_msgs/String   (JSON list of detections)
  pub  /vision/markers/debug    sensor_msgs/Image
"""
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2

try:
    import pytesseract
    TESSERACT = True
except ImportError:
    TESSERACT = False

# ── Tunable parameters ────────────────────────────────────────────────────────
MIN_AREA_PX  = 1000   # ignore smaller blobs (noise / very distant markers)
ASPECT_TOL   = 0.35   # how far from square (1.0) the bounding box can be
BLACK_THRESH = 0.40   # min black fraction in the two dark triangles
LARGE_AREA_PX = 12000  # bbox px² threshold between large (1.2 m) and small (0.6 m)

# Extreme-value thresholds for isolating the marker from the background.
# Marker white (printed paper in sunlight) ≥ 200; marker black ink ≤ 40.
# Mojave desert background sits in the 60–175 gray range and triggers neither.
WHITE_BRIGHT = 185   # pixels brighter than this are marker white
BLACK_DARK   = 50    # pixels darker than this are marker black
CLOSE_K      = 12    # kernel size for morphological close that bridges the gap
# ─────────────────────────────────────────────────────────────────────────────

NORM = 200  # warp candidates to this square size for pattern analysis


def _order_pts(pts):
    """Return 4 points in order: top-left, top-right, bottom-right, bottom-left."""
    pts = pts.reshape(4, 2).astype(np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).flatten()
    return np.array([pts[s.argmin()], pts[d.argmin()],
                     pts[s.argmax()], pts[d.argmax()]], dtype=np.float32)


def _warp(gray, pts):
    """Perspective-warp detected quad to a NORM×NORM square."""
    src = _order_pts(pts)
    dst = np.array([[0, 0], [NORM-1, 0], [NORM-1, NORM-1], [0, NORM-1]], np.float32)
    return cv2.warpPerspective(gray, cv2.getPerspectiveTransform(src, dst), (NORM, NORM))


def _triangle_masks():
    """Build pixel masks for each of the 4 triangles inside a NORM×NORM square."""
    N, c = NORM, NORM // 2
    masks = {}
    for name, corners in [
        ('top',    [(0, 0), (N-1, 0), (c, c)]),
        ('bottom', [(0, N-1), (N-1, N-1), (c, c)]),
        ('left',   [(0, 0), (0, N-1), (c, c)]),
        ('right',  [(N-1, 0), (N-1, N-1), (c, c)]),
    ]:
        m = np.zeros((N, N), np.uint8)
        cv2.fillPoly(m, [np.array(corners, np.int32)], 255)
        masks[name] = m
    return masks


_MASKS = _triangle_masks()


def _check_pattern(warped_gray):
    """
    Check whether the warped square matches the target marker pattern.
    Returns (is_valid, rotation_angle) where rotation_angle is 0 or 90.
    0  → top/bottom are black (standard)
    90 → left/right are black (marker rotated 90°)
    """
    _, binary = cv2.threshold(warped_gray, 127, 255, cv2.THRESH_BINARY_INV)  # black pixels = 255

    def bf(mask_name):
        m = _MASKS[mask_name]
        total = np.count_nonzero(m)
        return np.count_nonzero(binary & m) / total if total else 0

    tb, bb, lb, rb = bf('top'), bf('bottom'), bf('left'), bf('right')

    if tb > BLACK_THRESH and bb > BLACK_THRESH and lb < 0.35 and rb < 0.35:
        return True, 0
    if lb > BLACK_THRESH and rb > BLACK_THRESH and tb < 0.35 and bb < 0.35:
        return True, 90
    return False, 0


def _read_number(warped_gray, angle):
    """Crop the white number triangle and OCR it. Returns digit string or None."""
    if not TESSERACT:
        return None
    # Rotate so number triangle is always on the right
    if angle == 90:
        warped_gray = cv2.rotate(warped_gray, cv2.ROTATE_90_CLOCKWISE)
    # Crop right half (the white triangle with the number)
    roi = warped_gray[:, NORM // 2:]
    # Upscale + threshold for better OCR
    roi = cv2.resize(roi, (roi.shape[1] * 4, roi.shape[0] * 4))
    _, roi = cv2.threshold(roi, 127, 255, cv2.THRESH_BINARY)
    # Try all 4 rotations, take whichever gives a valid digit
    for rot in [None, cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180, cv2.ROTATE_90_COUNTERCLOCKWISE]:
        img = cv2.rotate(roi, rot) if rot is not None else roi
        text = pytesseract.image_to_string(
            img, config='--psm 7 --oem 3 -c tessedit_char_whitelist=0123456789'
        ).strip()
        if text.isdigit() and 1 <= int(text) <= 10:
            return text
    return None


class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        self.create_subscription(Image, '/camera/image_raw', self._cb, 1)
        self.pub = self.create_publisher(String, '/vision/markers', 10)
        self.pub_debug = self.create_publisher(Image, '/vision/markers/debug', 1)
        if not TESSERACT:
            self.get_logger().warning('pytesseract not installed — numbers will not be read. '
                                      'Run: pip install pytesseract && sudo apt install tesseract-ocr')
        self.get_logger().info('Marker detector started')

    def _cb(self, msg):
        yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height * 3 // 2, msg.width)
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        bright = cv2.threshold(blurred, WHITE_BRIGHT, 255, cv2.THRESH_BINARY)[1]
        dark   = cv2.threshold(blurred, BLACK_DARK,   255, cv2.THRESH_BINARY_INV)[1]
        combined = cv2.bitwise_or(bright, dark)
        solid = cv2.morphologyEx(combined, cv2.MORPH_CLOSE,
                                 np.ones((CLOSE_K, CLOSE_K), np.uint8))

        contours, _ = cv2.findContours(solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cx, cy = msg.width // 2, msg.height // 2
        detections = []
        debug = bgr.copy()

        seen = set()  # deduplicate overlapping contours

        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA_PX:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            if len(approx) != 4:
                continue
            x, y, w, h = cv2.boundingRect(approx)
            if not (1 - ASPECT_TOL < w / max(h, 1) < 1 + ASPECT_TOL):
                continue
            # Deduplicate by rounding bounding-box centre to nearest 20px
            key = (round((x + w/2) / 20), round((y + h/2) / 20))
            if key in seen:
                continue
            seen.add(key)

            warped = _warp(gray, approx.reshape(4, 2))
            valid, angle = _check_pattern(warped)
            if not valid:
                continue

            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            px = int(M['m10'] / M['m00'])
            py = int(M['m01'] / M['m00'])

            bbox_area = w * h
            size = 'large' if bbox_area >= LARGE_AREA_PX else 'small'
            number = _read_number(warped, angle) if size == 'small' else None

            det = {'offset_x': px - cx, 'offset_y': py - cy,
                   'area': bbox_area, 'size': size, 'number': number}
            detections.append(det)

            color = (0, 255, 0) if size == 'large' else (0, 165, 255)
            cv2.drawContours(debug, [approx], -1, color, 2)
            cv2.circle(debug, (px, py), 8, (0, 0, 255), -1)
            label = f"{size} #{number}" if number else size
            cv2.putText(debug, label, (px + 10, py), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self.pub.publish(String(data=json.dumps(detections)))
        if detections:
            self.get_logger().info(f'Markers: {detections}')
        self._publish_debug(msg, debug)

    def _publish_debug(self, msg, debug):
        if self.pub_debug.get_subscription_count() == 0:
            return
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
    node = MarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
