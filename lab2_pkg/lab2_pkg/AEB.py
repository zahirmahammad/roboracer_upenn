import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class AEB(Node):
    def __init__(self):
        super().__init__("AEB_node")

        # start a publisher
        self.throttle_pub = self.create_publisher(Float32, "/autodrive/f1tenth_1/throttle_command", 100)
        # self.steering_pub = self.create_publisher(Float32, "/autodrive/f1tenth_1/steering_command", QoSProfile(depth=1))

        # start a subscriber {throttle}
        self.create_subscription(Float32, "/autodrive/f1tenth_1/throttle_command", self.throttle_sub_callback, 100)

        # start a subscriber {laser}
        self.create_subscription(LaserScan, "/autodrive/f1tenth_1/lidar", self.start_AEB, 10)
      
        self.vel_x = 0.0
        self.emergency_stop = False
        # self.prev_ranges = None
        # self.prev_time = None
        self.TTC_THRESH = 0.005


    def throttle_sub_callback(self, msg):
        self.vel_x = msg.data
        # print(self.vel_x)
        if self.emergency_stop:
            vel_0 = Float32()
            self.throttle_pub.publish(vel_0)


    def start_AEB(self, data):
        curr_ranges = np.array(data.ranges)
        # curr_time = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9

        # component of velocity in each direction
        thetas = np.linspace(start=0, stop=270, num=1081)

        # cos component
        v = self.vel_x * thetas

        # Get the TTC
        time_collision = curr_ranges / v

        print(f"Min dist: {np.min(curr_ranges)}")
        print(f"time to collision dist: {np.min(time_collision)}")

        # check {threshold dist, threshold collision} for emergency braking
        if np.min(time_collision) <= 0.01 and np.min(curr_ranges)<0.3:
            print("Activate Emergency stop")
            self.emergency_stop = True

        # if self.prev_ranges is not None and self.prev_time is not None:
        #     # Calculate change in lidar values
        #     delta_ranges = curr_ranges - self.prev_ranges
        #     # Calculate the change in time
        #     delta_time = curr_time - self.prev_time

        #     # Range Rate {change in range over time} [basically the speed]
        #     r_dot = delta_ranges / delta_time

        #     # print(f"Delta ranges: {np.min(delta_ranges)}")

        #     # time to collision (distance / speed)
        #     r_dot = np.nan_to_num(r_dot, nan=0.0)
        #     ttc = curr_ranges / np.maximum(-r_dot, 0)

        #     # print(f"r_dot: {np.min(r_dot)}")

        #     # print(f"Time to collision: {ttc}")

        #     min_ttc = np.nanmin(ttc)
        #     print(f"Min of Time: {min_ttc}")

        #     # if min_ttc < 1000 and min_ttc >= 0.5:
        #         # self.emergency_stop = False

        #     if min_ttc <= 0.05 and not math.isnan(min_ttc):
        #         self.emergency_stop = True

        # # Update previous ranges
        # self.prev_ranges = curr_ranges
        # self.prev_time = curr_time








def main():
    rclpy.init()
    aeb_node = AEB()
    rclpy.spin(aeb_node)

    aeb_node.destroy_node()
    aeb_node.shutdown()