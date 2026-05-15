import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pies_servo import user_main


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.create_subscription(Float32, '/servo/angle', self._cb, 10)
        self.get_logger().info('Servo node started — listening on /servo/angle')

    def _cb(self, msg):
        user_main.on_angle_command(msg.data)


def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        user_main.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()
