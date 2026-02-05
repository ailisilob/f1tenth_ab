import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        
        # 1. PID Parameters (Tuned for Simulation)
        # Increase Kp for faster response, Kd to dampen oscillations
        self.kp = 2.5 
        self.kd = 0.09
        self.ki = 0.001 
        
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Target distance from the wall (in meters)
        self.desired_distance = 1.0  

        # 2. Subscribers and Publishers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def get_range(self, range_data, angle, msg):
        """
        Helper to get distance at a specific angle, handling invalid data.
        """
        index = int((angle - msg.angle_min) / msg.angle_increment)
        index = max(0, min(index, len(range_data) - 1))
        
        dist = range_data[index]
        if np.isnan(dist) or np.isinf(dist):
            return 4.0 # Default to a safe distance if sensor fails
        return dist

    def scan_callback(self, msg):
        """
        Main logic for wall following. 
        Note: Positive angles are Left, Negative are Right.
        """
        # Angles for Left Wall Follow (adjust to negative for Right Wall Follow)
        angle_b = np.pi / 2          # 90 degrees (perpendicular to car)
        angle_a = np.pi / 4          # 45 degrees (forward-left)
        theta = abs(angle_a - angle_b)    
        
        a = self.get_range(msg.ranges, angle_a, msg)
        b = self.get_range(msg.ranges, angle_b, msg)

        # STEP 1: Calculate alpha (angle between car orientation and the wall)
        # alpha = atan2(a*cos(theta) - b, a*sin(theta))
        alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta))

        # STEP 2: Calculate current and projected future distance to wall
        current_dist = b * np.cos(alpha)
        
        # Lookahead distance L helps anticipate turns. Increase this to stay further from walls.
        lookahead_L = 1.2 
        future_dist = current_dist + lookahead_L * np.sin(alpha)

        # STEP 3: Calculate error for PID
        # Error = (Projected distance) - (Desired distance)
        error = future_dist - self.desired_distance
        
        self.integral += error
        derivative = error - self.prev_error
        
        # Prevent integral windup
        self.integral = np.clip(self.integral, -1.0, 1.0)
        
        # Calculate steering command
        steering_angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error

        # STEP 4: Clip and Publish
        # Physical steering limit for F1TENTH is roughly 0.41 radians (~23 degrees)
        steering_angle = np.clip(steering_angle, -0.41, 0.41)
        self.publish_drive(steering_angle)

    def publish_drive(self, steering_angle):
        drive_msg = AckermannDriveStamped()
        
        # Dynamic Speed Control: Go fast on straights, slow down for sharp turns
        abs_steer = abs(steering_angle)
        
        if abs_steer < np.radians(5):    # Very straight
            drive_msg.drive.speed = 4.0 
        elif abs_steer < np.radians(15): # Minor correction
            drive_msg.drive.speed = 2.5
        else:                            # Sharp turn
            drive_msg.drive.speed = 1.2
            
        drive_msg.drive.steering_angle = steering_angle
        self.pub_drive.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    rclpy.spin(node)
    rclpy.shutdown()