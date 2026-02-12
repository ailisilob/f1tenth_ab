import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class MultiBubbleFollow(Node):
    def __init__(self):
        super().__init__('multi_bubble_follow')

        # ===== PID =====
        self.kp = 0.95
        self.kd = 0.05
        self.ki = 0.001

        self.prev_error = 0.0
        self.integral = 0.0

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.pub_drive = self.create_publisher(
            AckermannDriveStamped, '/drive', 10
        )

    # --------------------------------------------------
    def get_range(self, ranges, angle, msg):
        idx = int((angle - msg.angle_min) / msg.angle_increment)
        idx = np.clip(idx, 0, len(ranges) - 1)
        d = ranges[idx]
        if np.isnan(d) or np.isinf(d):
            return 4.0
        return d

    # --------------------------------------------------
    def bubble_stats(self, msg, ang_min, ang_max, samples=20):
        vals = []
        for ang in np.linspace(ang_min, ang_max, samples):
            vals.append(self.get_range(msg.ranges, ang, msg))
        return np.min(vals), np.mean(vals)

    # --------------------------------------------------
    def scan_callback(self, msg):
        # ===== Bubble sectors =====
        left_min, left_mean = self.bubble_stats(
            msg, np.radians(30), np.radians(70)
        )
        center_min, center_mean = self.bubble_stats(
            msg, np.radians(-8), np.radians(8)
        )
        right_min, right_mean = self.bubble_stats(
            msg, np.radians(-70), np.radians(-30)
        )

        # >>> Left-front constraint bubble <<<
        lf_min, lf_mean = self.bubble_stats(
            msg, np.radians(10), np.radians(30)
        )

        # ===== Parameters =====
        emergency_dist = 0.8
        safe_dist = 1.4
        inflate_bias = 0.4

        # ==================================================
        # Decision: which direction
        # ==================================================
        if center_min < emergency_dist:
            if left_min > right_min:
                desired_angle = np.radians(35)
            else:
                desired_angle = np.radians(-35)
        else:
            center_score = center_mean - inflate_bias * (1.0 / (center_min + 1e-3))
            left_score = left_mean
            right_score = right_mean

            if center_score > safe_dist:
                desired_angle = 0.0
            elif left_score > right_score:
                desired_angle = np.radians(25)
            else:
                desired_angle = np.radians(-25)

        # ==================================================
        # Geometric constraint (left turn curvature limit)
        # ==================================================
        target_angle = desired_angle

        if desired_angle > 0.0:
            lf_min_clipped = np.clip(lf_min, 0.4, 2.0)

            max_left_angle = np.interp(
                lf_min_clipped,
                [0.4, 2.0],
                [np.radians(8), np.radians(30)]
            )

            target_angle = min(desired_angle, max_left_angle)

        # ==================================================
        # PID
        # ==================================================
        error = target_angle
        self.integral += error
        derivative = error - self.prev_error

        steering = (
            self.kp * error +
            self.kd * derivative +
            self.ki * self.integral
        )

        self.prev_error = error
        steering = np.clip(steering, -0.41, 0.41)

        self.publish_drive(steering)

    # --------------------------------------------------
    def publish_drive(self, steering):
        msg = AckermannDriveStamped()
        a = abs(steering)

        if a < np.radians(5):
            msg.drive.speed = 1.5
        elif a < np.radians(12):
            msg.drive.speed = 1.0
        else:
            msg.drive.speed = 0.5

        msg.drive.steering_angle = steering
        self.pub_drive.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiBubbleFollow()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
