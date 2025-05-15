# ab1_pkg/scripts/relay.py
def main():
    print("Hello from relay!")


import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

class Relay(Node):
    def __init__(self):
        super().__init__("relay_node")

        # start a subscriber
        self.create_subscription(AckermannDriveStamped, "drive", self.relay_callback, 10)

        # Publisher
        self.relay_pub = self.create_publisher(AckermannDriveStamped, "drive_relay", 10)

    def relay_callback(self, data):
        speed = 3 * data.drive.speed
        angle = 3 * data.drive.steering_angle

        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle

        self.relay_pub.publish(msg)

        self.get_logger().info(f"Published speed: {speed}, angle: {angle}")


def main():
    rclpy.init()
    relay = Relay()
    rclpy.spin(relay)

    relay.destroy_node()
    rclpy.shutdown()

