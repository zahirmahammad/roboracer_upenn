import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
# from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.scan_callback,
            10)

        self.throt_pub = self.create_publisher(
            Float32,
            "/autodrive/f1tenth_1/throttle_command",
            10)
        
        self.steer_pub = self.create_publisher(
            Float32,
            "/autodrive/f1tenth_1/steering_command",
            10)

        # PID gains
        self.kp = 1.2
        self.kd = 0.3
        self.ki = 0.0

        print("WallFollowi in init. fuck this")
        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        # Desired distance to the wall (left side)
        self.desired_dist = 1.0

        # Lookahead for prediction (distance in meters the car travels)
        self.lookahead_dist = 1.0

    def get_range(self, range_data, angle, msg):
        """
        Get the lidar range at a specific angle.
        Angle is in radians. Positive = left, negative = right.
        """
        index = int((angle - msg.angle_min) / msg.angle_increment)
        if 0 <= index < len(range_data):
            dist = range_data[index]
            if np.isfinite(dist):
                return dist
        return float('inf')

    def get_error(self, range_data, msg):
        """
        Estimate the future distance error to the left wall.
        """
        theta = np.radians(50)  # angle between a and b
        a = self.get_range(range_data, np.radians(70), msg)  # front-left
        b = self.get_range(range_data, np.radians(110), msg)  # left

        if np.isinf(a) or np.isinf(b):
            return 0.0

        # Calculate angle to the wall (alpha)
        num = a * np.cos(theta) - b
        den = a * np.sin(theta)
        if den == 0:
            alpha = 0
        else:
            alpha = np.arctan2(num, den)

        # Projected distance to the wall after lookahead
        Dt = b * np.cos(alpha)
        Dt1 = Dt + self.lookahead_dist * np.sin(alpha)

        error = self.desired_dist - Dt1
        return error

    def pid_control(self, error):
        """
        PID controller to compute steering angle.
        """
        self.integral += error
        derivative = error - self.prev_error
        control = self.kp * error + self.kd * derivative + self.ki * self.integral
        self.prev_error = error
        return control

    def compute_velocity(self, steering_angle):
        angle_deg = np.abs(np.degrees(steering_angle))
        if angle_deg < 10:
            return 1.5
        elif angle_deg < 20:
            return 1.0
        else:
            return 0.5

    def scan_callback(self, msg):
        error = self.get_error(msg.ranges, msg)
        steering_angle = self.pid_control(error)
        speed = self.compute_velocity(steering_angle)

        # drive_msg = AckermannDriveStamped()
        # drive_msg.drive.steering_angle = steering_angle
        # drive_msg.drive.speed = speed
        # self.drive_pub.publish(drive_msg)
        
        throttle_msg = Float32()
        throttle_msg.data = speed
        steering_msg = Float32()
        steering_msg.data = steering_angle

        # print(f"Steering Angle: {steering_angle:.2f}, Speed: {speed:.2f}, Error: {error:.2f}")

        # Publish the throttle and steering commands
        self.throt_pub.publish(throttle_msg)
        self.steer_pub.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()
    print("WallFollowing Node Initialized")
    rclpy.spin(wall_follow_node)

    # wall_follow_node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
