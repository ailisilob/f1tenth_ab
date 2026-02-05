#!/usr/bin/env python3

import rclpy

from rclpy.node import Node



import numpy as np

# TODO: include needed ROS msg type headers and libraries

from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive





class SafetyNode(Node):

    """

    The class that handles emergency braking.

    """

    def __init__(self):

        super().__init__('safety_node')

        """

        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.



        You should also subscribe to the /scan topic to get the LaserScan messages and

        the /ego_racecar/odom topic to get the current speed of the vehicle.



        The subscribers should use the provided odom_callback and scan_callback as callback methods



        NOTE that the x component of the linear velocity in odom is the speed

        """

        self.speed = 0.

        # TODO: create ROS subscribers and publishers.

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)



    def odom_callback(self, odom_msg):

        # TODO: update current speed

        self.speed = odom_msg.twist.twist.linear.x



    def scan_callback(self, scan_msg):

        # TODO: calculate TTC

        ranges = np.array(scan_msg.ranges)

        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        r_dot = self.speed * np.cos(angles)

        ttc = ranges / np.maximum(r_dot, 1e-6)

        if np.any((ttc<0.5) & (r_dot>0)):

            self.publish_brake()

        # TODO: publish command to brake



    def publish_brake(self):

        brake_msg = AckermannDriveStamped()

        brake_msg.drive.speed = 0.0

        self.drive_publisher.publish(brake_msg)

        self.get_logger().info('Emergency Brake Activated!')





def main(args=None):

    rclpy.init(args=args)

    safety_node = SafetyNode()

    rclpy.spin(safety_node)



    # Destroy the node explicitly

    # (optional - otherwise it will be done automatically

    # when the garbage collector destroys the node object)

    safety_node.destroy_node()

    rclpy.shutdown()





if __name__ == '__main__':

    main()