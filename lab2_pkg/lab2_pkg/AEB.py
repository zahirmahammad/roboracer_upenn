import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import numpy as np

class AEB(Node):
    def __init__(self):
        super().__init__("AEB_node")

        # start a publisher
        self.throttle_pub = self.create_publisher(Float32, "/autodrive/f1tenth_1/throttle_command", QoSProfile(depth=1))
        self.steering_pub = self.create_publisher(Float32, "/autodrive/f1tenth_1/steering_command", QoSProfile(depth=1))


        # start a subscriber
        self.create_subscription(LaserScan, "/autodrive/f1tenth_1/lidar", self.start_AEB, 1)


        # Throttle msg
        throttle_msg = Float32()
        
        # steering msg
        steering_msg = Float32()

        self.prev_ranges = None
        self.prev_time = None
    
    def start_AEB(self, data):
        curr_ranges = np.array(data.ranges)
        curr_time = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9

        if self.prev_ranges is not None and self.prev_time is not None:
            # Calculate change in lidar values
            delta_ranges = curr_ranges - self.prev_ranges
            # Calculate the change in time
            delta_time = curr_time - self.prev_time

            # Range Rate {change in range over time} [basically the speed]
            r_dot = delta_ranges / delta_time

            # time to collision (distance / speed)
            ttc = curr_ranges / np.maximum(-r_dot, 0)

            print(f"Time to collision: {np.minimum(ttc)}")


        # Update previous ranges
        self.prev_ranges = curr_ranges
        self.prev_time = curr_time








def main():
    rclpy.init()
    aeb_node = AEB()
    rclpy.spin(aeb_node)

    aeb_node.destroy_node()
    aeb_node.shutdown()