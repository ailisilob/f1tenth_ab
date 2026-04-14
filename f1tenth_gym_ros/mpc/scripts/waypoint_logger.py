#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import os
import math
from nav_msgs.msg import Odometry

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        self.output_file = '/sim_ws/src/mpc/scripts/levine_raw.csv'
        self.min_dist = 0.05  # 每5cm记录一个点，转弯更密
        self.last_pos = None
        self.waypoints = []
        
        self.sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        self.get_logger().info(f'Recording to: {self.output_file}')
        self.get_logger().info('开始开车！慢慢绕一圈，转弯处稳一点')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.last_pos is None:
            self.waypoints.append([x, y])
            self.last_pos = (x, y)
            return
        
        dist = math.sqrt((x - self.last_pos[0])**2 + (y - self.last_pos[1])**2)
        if dist >= self.min_dist:
            self.waypoints.append([x, y])
            self.last_pos = (x, y)
            with open(self.output_file, 'w', newline='') as f:
                csv.writer(f).writerows(self.waypoints)
            if len(self.waypoints) % 100 == 0:
                self.get_logger().info(f'已记录 {len(self.waypoints)} 个点')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WaypointLogger())

if __name__ == '__main__':
    main()
