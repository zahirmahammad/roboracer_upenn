import rclpy
import rclpy.logging
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class Talker(Node):
    def __init__(self):
        print("Hi from Talker!")
        super().__init__('talker_node')

        self.declare_parameter("v", 1.0)
        self.declare_parameter("d", 0.0)


        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)
        time_ = 0.5
        self.timer = self.create_timer(time_, self.publish_drive)

    def publish_drive(self):
        speed = self.get_parameter("v").get_parameter_value().double_value
        angle = self.get_parameter("d").get_parameter_value().double_value

        # publish Ackermannmsg
        msg = AckermannDriveStamped()
        msg.drive.speed =  speed
        msg.drive.steering_angle = angle
        self.publisher.publish(msg)

        self.get_logger().info(f"Published speed:{speed}, angle:{angle}")




def main():
    rclpy.init()
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()