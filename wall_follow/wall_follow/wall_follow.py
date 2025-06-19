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
        self.kp = 0.9
        self.kd = 0.2
        self.ki = 0.0

        print("WallFollowi in init. fuck this")
        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        # Desired distance to the wall (left side)
        self.desired_dist = 0.7

        # Lookahead for prediction (distance in meters the car travels)
        self.lookahead_dist = 1.2

        # index of ray b
        # angle at which b will be = 45degrees
        # 0.25 = resol of hukoyo lidar
        self.ind_b = int(45 / 0.25)
        
        # index of ray a (50 degrees angle with b)
        self.ind_a = int((45+50) / 0.25)


    def get_error(self, range_data, msg):
        """
        Estimate the future distance error to the left wall.
        """
        theta = np.radians(50)  # angle between a and b
        a = range_data[self.ind_a]
        b = range_data[self.ind_b]

        # print(f"Distance at a: {a}")
        # print(f"Distance at b: {b}")


        # if np.isinf(a) or np.isinf(b):
        #     return 0.0

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
            return 0.35
        elif angle_deg < 20:
            return 0.2
        else:
            return 0.15

    def scan_callback(self, msg):
        error = self.get_error(msg.ranges, msg)
        # print(error)
        steering_angle = self.pid_control(error)
        print(steering_angle)
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
