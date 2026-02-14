#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


class ReactiveFollowGap(Node):
    
    def __init__(self):
        super().__init__('reactive_node')
        
        # Subscribe to LiDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Publish drive commands
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        
        # Parameter settings
        self.BUBBLE_RADIUS = 0.3  # Safety bubble radius (meters)
        self.MAX_SPEED = 1.0      # Maximum speed
        self.MIN_SPEED = 0.5      # Minimum speed
    
    def preprocess_lidar(self, ranges):
        """
        Step 1: Preprocess LiDAR data
        """
        proc_ranges = np.array(ranges)
        # Set invalid values to 0
        proc_ranges[np.isinf(proc_ranges)] = 0
        proc_ranges[np.isnan(proc_ranges)] = 0
        return proc_ranges
    
    def find_max_gap(self, free_space_ranges):
        """
        Step 4: Find the maximum gap
        """
        # Find all non-zero indices
        non_zero = np.nonzero(free_space_ranges)[0]
        if len(non_zero) == 0:
            return 0, 0
        
        # Find continuous gaps
        gaps = []
        start = non_zero[0]
        for i in range(1, len(non_zero)):
            if non_zero[i] - non_zero[i-1] > 1:
                gaps.append((start, non_zero[i-1]))
                start = non_zero[i]
        gaps.append((start, non_zero[-1]))
        
        # Return the longest gap
        if len(gaps) == 0:
            return 0, 0
        max_gap = max(gaps, key=lambda x: x[1] - x[0])
        return max_gap[0], max_gap[1]
    
    def find_best_point(self, start_i, end_i, ranges):
        """
        Step 5: Find the best point in the gap
        """
        # Simple approach: return the furthest point in the gap
        if end_i - start_i < 1:
            return len(ranges) // 2
        
        gap_ranges = ranges[start_i:end_i+1]
        best_idx = np.argmax(gap_ranges)
        return start_i + best_idx
    
    def lidar_callback(self, data):
        """
        Main callback function
        """
        # Step 1: Preprocess
        ranges = self.preprocess_lidar(data.ranges)
        
        # Step 2: Find the closest point
        min_idx = np.argmin(ranges[ranges > 0]) if np.any(ranges > 0) else len(ranges) // 2
        min_dist = ranges[min_idx]
        
        # Step 3: Draw safety bubble around the closest point
        if min_dist > 0:
            # Calculate the angular range covered by the bubble
            bubble_size = int(np.ceil(self.BUBBLE_RADIUS / (min_dist * data.angle_increment)))
            start = max(0, min_idx - bubble_size)
            end = min(len(ranges), min_idx + bubble_size)
            ranges[start:end] = 0
        
        # Step 4: Find the maximum gap
        start_i, end_i = self.find_max_gap(ranges)
        
        # Step 5: Find the best point in the gap
        best_point = self.find_best_point(start_i, end_i, ranges)
        
        # Step 6: Calculate steering angle and publish command
        # Calculate the angle to the target point
        num_ranges = len(ranges)
        angle = data.angle_min + best_point * data.angle_increment
        
        # Adjust speed based on steering angle
        if abs(angle) > 0.3:  # Large turn
            speed = self.MIN_SPEED
        else:  # Straight
            speed = self.MAX_SPEED
        
        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()