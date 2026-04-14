import rclpy
from rclpy.node import Node
import csv
import os
import math
from nav_msgs.msg import Odometry

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        self.output_file = os.path.expanduser(
            '~/f1tenth_ws/src/pure_pursuit/waypoints/levine_waypoints.csv')
        self.min_dist = 0.1  # capture a waypoint every 10cm
        self.last_pos = None
        self.waypoints = []
        
        # ensure directory exists
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        
        self.sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.get_logger().info(f'Recording waypoints to: {self.output_file}')
        self.get_logger().info('Drive the car around the track now!')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.last_pos is None:
            self.record_point(x, y)
            self.last_pos = (x, y)
            return
        
        dist = math.sqrt((x - self.last_pos[0])**2 + (y - self.last_pos[1])**2)
        if dist >= self.min_dist:
            self.record_point(x, y)
            self.last_pos = (x, y)

    def record_point(self, x, y):
        self.waypoints.append([x, y])
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(self.waypoints)
        if len(self.waypoints) % 50 == 0:
            self.get_logger().info(f'Recorded {len(self.waypoints)} waypoints...')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()