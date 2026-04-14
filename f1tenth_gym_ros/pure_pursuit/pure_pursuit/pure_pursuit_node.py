import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # --- param ---
        self.L = 1.0       
        self.velocity = 1.5   
        self.wheelbase = 0.33 
        
        waypoints_path = os.path.expanduser(
            '/sim_ws/src/pure_pursuit/waypoints/levine_waypoints.csv')
        
        # --- load waypoints ---
        self.waypoints = self.load_waypoints(waypoints_path)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
        # --- Subscribers ---
        self.pose_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        # --- Publishers ---
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.waypoints_pub = self.create_publisher(
            MarkerArray, '/waypoints_viz', 10)
        self.target_pub = self.create_publisher(
            Marker, '/target_waypoint_viz', 10)
        
        # publish all waypoints visualization at 1Hz
        self.create_timer(1.0, self.publish_all_waypoints)
        
        self.get_logger().info('Pure Pursuit node started!')

    def load_waypoints(self, path):
        waypoints = []
        try:
            with open(path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 2:
                        try:
                            waypoints.append([float(row[0]), float(row[1])])
                        except ValueError:
                            continue
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
        return np.array(waypoints)

    def odom_callback(self, msg: Odometry):
        # obtain current position and orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.run_pure_pursuit(x, y, yaw)

    def run_pure_pursuit(self, x, y, yaw):
        if len(self.waypoints) == 0:
            return
        
        # 1. find nearest waypoint index
        dists = np.linalg.norm(self.waypoints - np.array([x, y]), axis=1)
        nearest_idx = int(np.argmin(dists))
        
        # 2. find lookahead point at least L away
        target_point = None
        target_idx = nearest_idx
        n = len(self.waypoints)
        
        for i in range(n):
            idx = (nearest_idx + i) % n
            dist = np.linalg.norm(self.waypoints[idx] - np.array([x, y]))
            if dist >= self.L:
                target_point = self.waypoints[idx]
                target_idx = idx
                break
        
        if target_point is None:
            target_point = self.waypoints[(nearest_idx + 1) % n]
            target_idx = (nearest_idx + 1) % n
        
        # 3. transform target point to local frame
        dx = target_point[0] - x
        dy = target_point[1] - y
        local_y = -dx * np.sin(yaw) + dy * np.cos(yaw)
        
        # 4. calculate curvature γ = 2|y| / L²
        curvature = 2.0 * local_y / (self.L ** 2)
        
        # 5. steering angle = arctan(γ * wheelbase)
        steering_angle = float(np.arctan(curvature * self.wheelbase))
        steering_angle = float(np.clip(steering_angle, -0.4, 0.4))
        
        # 6. slow down if steering is sharp
        speed = self.velocity
        if abs(steering_angle) > 0.2:
            speed = self.velocity * 0.5
        elif abs(steering_angle) > 0.1:
            speed = self.velocity * 0.75
        
        # 7. publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)
        
        # 8. visualization
        self.publish_target_waypoint(target_point)

    def publish_all_waypoints(self):
        """all waypoints → green spheres"""
        if len(self.waypoints) == 0:
            return
        marker_array = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wp[0])
            m.pose.position.y = float(wp[1])
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.r = 0.0
            m.color.g = 1.0  # green
            m.color.b = 0.0
            m.color.a = 0.8
            marker_array.markers.append(m)
        self.waypoints_pub.publish(marker_array)

    def publish_target_waypoint(self, point):
        """target waypoint → red sphere"""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(point[0])
        m.pose.position.y = float(point[1])
        m.pose.position.z = 0.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.r = 1.0  # red
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        self.target_pub.publish(m)

def euler_from_quaternion(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return 0.0, 0.0, yaw
    


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()