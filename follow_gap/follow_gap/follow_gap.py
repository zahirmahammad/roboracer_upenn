import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')

        # TODO: Subscribe to LIDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.lidar_callback,
            10)
        
        # TODO: Publish to drive
        self.throt_pub = self.create_publisher(
            Float32,
            "/autodrive/f1tenth_1/throttle_command",
            10)
        
        self.steer_pub = self.create_publisher(
            Float32,
            "/autodrive/f1tenth_1/steering_command",
            10)
        
        self.safety_bubble = 1.5  # Safety bubble in meters
        self.max_distance = 2.3  # Maximum distance to consider for free space in meters
        self.max_gap_points = 4  # Minimum number of points in a gap to consider it valid
        self.back_left = None
        self.back_right = None
        self.back_hit_thresh = 0.2




    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        
        proc_ranges = np.array(ranges).copy()

        # tweak 3 - find and copy back rays (tweak 1 is overwriting the back rays)
        # ----------------------------------------------------
        self.back_left = proc_ranges[int(225/0.25):].copy()
        self.back_right = proc_ranges[:int(45/0.25)].copy()


        # block the back array of the LiDAR scan
        # ----------------------------------------------------
        proc_ranges[:int(30/0.25)] = 0.0    # took off 15 degrees to only consider back rays
        proc_ranges[int(215/0.25):] = 0.0
        # ----------------------------------------------------



        # tweak 1 - get the nearest point; draw a safety bubble around it; set all points to zero
        # -------------------------------------------------------------------------
        # find the x and y coordinates of all points in the LiDAR scan
        # create an array of length similar to ranges
        indices = np.arange(len(ranges))
        angles_array = np.deg2rad(indices * 0.25)  # angles in radians
        x_coords = ranges * np.cos(angles_array)
        y_coords = ranges * np.sin(angles_array)

        # find min point in the LiDAR scan
        min_index = np.argmin(ranges)
        x_min = x_coords[min_index]
        y_min = y_coords[min_index]
        # find all points within the safety bubble
        distance_from_min = np.sqrt((x_coords - x_min) ** 2 + (y_coords - y_min) ** 2)

        # set all points within the safety bubble to zero
        proc_ranges[distance_from_min < self.safety_bubble] = 0.0
        # -------------------------------------------------------------------------
        
        # now, the non-zero points are the ones with free space
        # tweak 2 - find disparities and set some points to zero
        # -------------------------------------------------------------------------

        threshold = 1.0

        for i in range(1, len(ranges) - 1):
            if ranges[i] == 0.0 or ranges[i-1] == 0.0:
                continue

            disparity = ranges[i] - ranges[i-1]
            if abs(disparity) > threshold:
                # Choose the closer distance
                close_idx = i if disparity < 0 else i - 1
                close_range = ranges[close_idx]

                # Angular width to cover the car width at this range
                print(f"close_range: {close_range}")
                angle_width_rad = np.arcsin((0.8 / 2) / close_range)
                if np.isnan(angle_width_rad):
                    print("Angle width is NaN")
                    continue
                print(f"Angle width in radians: {angle_width_rad}")
                angle_width_deg = np.rad2deg(angle_width_rad)
                num_points = int(angle_width_deg / 0.25)

                # Fill in points on the "far" side with the closer distance
                if disparity > 0:  # right is farther
                    proc_ranges[close_idx:close_idx + num_points] = close_range
                else:  # left is farther
                    proc_ranges[close_idx - num_points:close_idx] = close_range
        # -------------------------------------------------------------------------





        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # find the max gap in the free space ranges (non-zero points)
        # get the boolean array of free space
        free_space = free_space_ranges >= self.max_distance
        # print number of ones in the free space array
        print(f"Free space ranges: {len(np.where(free_space)[0])} points")
        # print(f"Free space ranges threshold: {len(free_space)}")
        # find the indices where the value changes
        # value_changes = np.diff(free_space.astype(int))
        # value_changes = value_changes + 1 # as np.diff reduces the length by 1

        # find the start and end indices of the gaps
        def find_consecutive_trues_simple(arr):
            results = []
            i = 0
            while i < len(arr):
                if arr[i]:
                    start = i
                    count = 0
                    while i < len(arr) and arr[i]:
                        count += 1
                        i += 1
                    if count >= self.max_gap_points:
                        results.append((count, start))
                else:
                    i += 1
            return results
        consecutive_trues = find_consecutive_trues_simple(free_space)
        # get the maximum gap
        # if consecutive_trues:
        # max_gap = max(consecutive_trues, key=lambda x: free_space_ranges[x[1]:x[1] + x[0]].sum()/ x[0]) # find the one with largest gap
        try:
            max_gap = max(consecutive_trues, key=lambda x: x[0]) # find the one with largest gap
            start_i = max_gap[1]
            end_i = start_i + max_gap[0] - 1

            # get the angle to traverse using start and end indices
            angle = (start_i + end_i) / 2 * 0.25  # 0.25 is the angle resolution of the LiDAR
            

            # steering angle is relative to the front of the car
            steering_angle = angle - 135 # 135 is front of car

            return np.deg2rad(steering_angle)    # degrees
        
        except ValueError:  # No valid gaps found
            print("No valid gaps found")
            return 0.0  # No steering angle if no gap is found

    def compute_velocity(self, steering_angle):
        angle_deg = np.abs(np.degrees(steering_angle))
        if angle_deg < 10:
            return 0.33
        elif angle_deg < 20:
            return 0.28
        else:
            return 0.2

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 
        angle = self.find_max_gap(proc_ranges)
        print(f"Max gap angle: {angle} degrees")

        #Find the best point in the gap 

        #Publish Drive message
        vel = self.compute_velocity(angle)
        steering_msg = Float32()
        steering_msg.data = angle
        throttle_msg = Float32()
        throttle_msg.data = vel
        print(f"min of back left: {np.min(self.back_left)}; min of back right: {np.min(self.back_right)}")
        if np.any(self.back_left <= self.back_hit_thresh) or np.any(self.back_right <= self.back_hit_thresh):
            print("Back rays are blocked, move forward")
            steering_msg.data = 0.0
        self.steer_pub.publish(steering_msg)
        self.throt_pub.publish(throttle_msg)

def main(args=None):
    rclpy.init(args=args)
    print("Follow Gap Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()