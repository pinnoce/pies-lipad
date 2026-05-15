#!/usr/bin/env python3
"""Save one frame from /camera/image_raw to captured_frame.jpg."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FrameCapture(Node):
    def __init__(self):
        super().__init__('frame_capture')
        self.bridge = CvBridge()
        self.done = False
        self.create_subscription(Image, '/camera/image_raw', self._cb, 1)
        self.get_logger().info('Waiting for a frame...')

    def _cb(self, msg):
        if self.done:
            return
        import numpy as np
        # NV21: Y plane (H×W) followed by VU interleaved (H/2×W)
        yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height * 3 // 2, msg.width)
        img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
        path = '/home/lipad/pies-lipad/captured_frame.jpg'
        cv2.imwrite(path, img)
        self.get_logger().info(f'Saved {msg.width}x{msg.height} frame to {path}')
        self.done = True

def main():
    rclpy.init()
    node = FrameCapture()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
