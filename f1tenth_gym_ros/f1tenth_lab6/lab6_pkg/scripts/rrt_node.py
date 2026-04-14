"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math
from scipy.ndimage import binary_dilation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


# class def for tree nodes
# It's up to you if you want to use this
class RRTNode(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = 0.0  # only used in RRT*
        self.is_root = False


# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')

        # topics, not saved as attributes
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"

        # RRT / RRT* parameters
        self.max_iter       = 500
        self.eta            = 0.5      # steer step size (meters)
        self.goal_threshold = 0.3
        self.goal_bias      = 0.15
        self.goal_dist      = 2.0      # how far ahead to place goal (meters)
        self.search_radius  = 1.0      # RRT* neighbor search radius (meters)

        # pure pursuit
        self.lookahead = 1.0
        self.speed     = 1.0

        # occupancy grid parameters (local frame: x=forward, y=left)
        self.grid_res      = 0.1
        self.grid_w        = 6.0
        self.grid_h        = 8.0
        self.inflate_r     = 0.25

        self.grid_cols     = int(self.grid_w / self.grid_res)
        self.grid_rows     = int(self.grid_h / self.grid_res)
        self.origin_col    = self.grid_cols // 2

        # pre-build dilation structuring element once
        inflate_cells      = int(self.inflate_r / self.grid_res)
        y, x = np.ogrid[-inflate_cells:inflate_cells+1,
                        -inflate_cells:inflate_cells+1]
        self.struct        = (x*x + y*y) <= inflate_cells**2

        self.occupancy     = np.zeros((self.grid_rows, self.grid_cols), dtype=bool)
        self.scan_msg      = None

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            Odometry,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_

        # publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.occ_pub   = self.create_publisher(OccupancyGrid, '/rrt/occupancy_grid', 10)
        self.tree_pub  = self.create_publisher(MarkerArray, '/rrt/tree', 10)
        self.path_pub  = self.create_publisher(MarkerArray, '/rrt/path', 10)
        self.goal_pub  = self.create_publisher(Marker, '/rrt/goal', 10)

        self.get_logger().info('RRT* node started')

    # -----------------------------------------------------------------------
    # Occupancy grid helpers
    # -----------------------------------------------------------------------

    def local_to_grid(self, lx, ly):
        """Convert local frame coords to grid (row, col)."""
        row = int(lx / self.grid_res)
        col = int(ly / self.grid_res) + self.origin_col
        return row, col

    def in_grid(self, row, col):
        return 0 <= row < self.grid_rows and 0 <= col < self.grid_cols

    def is_occupied(self, lx, ly):
        row, col = self.local_to_grid(lx, ly)
        if not self.in_grid(row, col):
            return True
        return self.occupancy[row, col]

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        self.scan_msg = scan_msg

        # 1. vectorised laser point projection
        raw    = np.zeros((self.grid_rows, self.grid_cols), dtype=bool)
        angles = (scan_msg.angle_min +
                  np.arange(len(scan_msg.ranges)) * scan_msg.angle_increment)
        ranges = np.array(scan_msg.ranges)

        valid  = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        lx     = ranges[valid] * np.cos(angles[valid])
        ly     = ranges[valid] * np.sin(angles[valid])

        rows   = (lx / self.grid_res).astype(int)
        cols   = (ly / self.grid_res).astype(int) + self.origin_col

        mask   = ((rows >= 0) & (rows < self.grid_rows) &
                  (cols >= 0) & (cols < self.grid_cols))
        raw[rows[mask], cols[mask]] = True

        # 2. single-pass inflation via binary_dilation
        self.occupancy = binary_dilation(raw, structure=self.struct)

        # publish for visualization
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'
        msg.info.resolution = self.grid_res
        msg.info.width      = self.grid_cols
        msg.info.height     = self.grid_rows
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = -self.grid_w / 2.0
        msg.data = (self.occupancy.astype(np.int8) * 100).flatten().tolist()
        self.occ_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Pose callback: main RRT* loop
    # -----------------------------------------------------------------------

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        if self.scan_msg is None:
            return None

        goal = self.select_goal()
        self.publish_goal(goal)

        path = self.run_rrt_star(goal)

        if path is not None and len(path) > 1:
            self.publish_path(path)
            self.pure_pursuit(path)
        else:
            msg = AckermannDriveStamped()
            msg.drive.speed          = 0.3
            msg.drive.steering_angle = 0.0
            self.drive_pub.publish(msg)

        return None

    def select_goal(self):
        """Select a free goal point ahead in the local frame."""
        for angle_deg in [0, 10, -10, 20, -20, 30, -30, 45, -45]:
            angle = math.radians(angle_deg)
            gx    = self.goal_dist * math.cos(angle)
            gy    = self.goal_dist * math.sin(angle)
            if self.line_is_free(0.0, 0.0, gx, gy):
                return (gx, gy)
        return (self.goal_dist * 0.5, 0.0)

    def line_is_free(self, x0, y0, x1, y1):
        """
        Bresenham line collision check on the occupancy grid.
        Returns True if the line is collision-free.
        """
        r0, c0 = self.local_to_grid(x0, y0)
        r1, c1 = self.local_to_grid(x1, y1)

        dr  = abs(r1 - r0);  sr = 1 if r1 > r0 else -1
        dc  = abs(c1 - c0);  sc = 1 if c1 > c0 else -1
        err = dr - dc
        r, c = r0, c0

        while True:
            if not self.in_grid(r, c):
                return False
            if self.occupancy[r, c]:
                return False
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc;  r += sr
            if e2 <  dr:
                err += dr;  c += sc
        return True

    # -----------------------------------------------------------------------
    # RRT* Algorithm 6 from paper
    # -----------------------------------------------------------------------

    def run_rrt_star(self, goal):
        """
        RRT* main loop following Algorithm 6 (Karaman & Frazzoli 2011).
        Extends RRT with choose-best-parent and rewire steps.
        Returns path as list of (x, y) tuples, or None.
        """
        root          = RRTNode()
        root.x        = 0.0
        root.y        = 0.0
        root.is_root  = True
        root.parent   = None
        root.cost     = 0.0
        V = [root]
        E = []

        goal_nodes = []

        for _ in range(self.max_iter):
            # Algorithm 3 steps 1-3
            x_rand      = self.sample(goal)
            nearest_idx = self.nearest(V, x_rand)
            x_nearest   = V[nearest_idx]
            x_new       = self.steer(x_nearest, x_rand)
            if x_new is None:
                continue

            # collision check on the new edge
            if self.check_collision(x_nearest, x_new):
                continue

            # ── RRT* step 1: choose best parent ──────────────────────────
            # find all neighbors within search_radius
            X_near = self.near(V, x_new)

            # default parent is nearest
            x_min_idx = nearest_idx
            c_min     = self.cost(V, x_nearest) + self.line_cost(x_nearest, x_new)

            for near_idx in X_near:
                x_near = V[near_idx]
                c_new  = self.cost(V, x_near) + self.line_cost(x_near, x_new)
                if c_new < c_min and not self.check_collision(x_near, x_new):
                    x_min_idx = near_idx
                    c_min     = c_new

            x_new.parent = x_min_idx
            x_new.cost   = c_min
            V.append(x_new)
            new_idx = len(V) - 1
            E.append((x_min_idx, new_idx))

            # ── RRT* step 2: rewire ───────────────────────────────────────
            for near_idx in X_near:
                x_near  = V[near_idx]
                c_rewire = self.cost(V, x_new) + self.line_cost(x_new, x_near)
                if c_rewire < self.cost(V, x_near) and not self.check_collision(x_new, x_near):
                    # remove old parent edge from E
                    old_parent = x_near.parent
                    if old_parent is not None:
                        E = [e for e in E if not (e[0] == old_parent and e[1] == near_idx)]
                    # rewire
                    x_near.parent = new_idx
                    x_near.cost   = c_rewire
                    E.append((new_idx, near_idx))

            # check goal
            if self.is_goal(x_new, goal[0], goal[1]):
                goal_nodes.append(x_new)

        self.publish_tree(V, E)
        
        # find the optima with all nodes
        if len(goal_nodes) > 0:
            # find the least cost node with lamda
            best_goal_node = min(goal_nodes, key = lambda node: node.cost)
            return self.find_path(V, best_goal_node)
    

        return None

    def sample(self, goal=None):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        if goal is not None and np.random.rand() < self.goal_bias:
            return goal
        max_x = self.grid_h - self.grid_res
        max_y = (self.grid_cols // 2) * self.grid_res
        x     = np.random.uniform(0.0, max_x)
        y     = np.random.uniform(-max_y, max_y)
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        dists        = [(sampled_point[0] - n.x)**2 + (sampled_point[1] - n.y)**2
                        for n in tree]
        nearest_node = int(np.argmin(dists))
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        dx   = sampled_point[0] - nearest_node.x
        dy   = sampled_point[1] - nearest_node.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1e-6:
            return None
        scale        = min(self.eta, dist) / dist
        new_node     = RRTNode()
        new_node.x   = nearest_node.x + scale * dx
        new_node.y   = nearest_node.y + scale * dy
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        return not self.line_is_free(nearest_node.x, nearest_node.y,
                                     new_node.x,     new_node.y)

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        dist = math.sqrt((latest_added_node.x - goal_x)**2 +
                         (latest_added_node.y - goal_y)**2)
        return dist < self.goal_threshold

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        node = latest_added_node
        while node is not None:
            path.append((node.x, node.y))
            if node.parent is None:
                break
            node = tree[node.parent]
        path.reverse()
        return path

    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        # cost is stored directly on the node (accumulated path length from root)
        return node.cost

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        r2           = self.search_radius ** 2
        neighborhood = [i for i, n in enumerate(tree)
                        if (n.x - node.x)**2 + (n.y - node.y)**2 <= r2]
        return neighborhood

    # -----------------------------------------------------------------------
    # Pure Pursuit trajectory follower
    # -----------------------------------------------------------------------

    def pure_pursuit(self, path):
        """Follow path using pure pursuit with upsampled waypoints."""
        dense = [path[0]]
        for i in range(1, len(path)):
            p0, p1 = path[i-1], path[i]
            dx, dy = p1[0]-p0[0], p1[1]-p0[1]
            seg    = math.sqrt(dx*dx + dy*dy)
            steps  = max(int(seg / 0.05), 1)
            for j in range(1, steps+1):
                t = j / steps
                dense.append((p0[0]+t*dx, p0[1]+t*dy))

        target = dense[-1]
        for pt in dense:
            if math.sqrt(pt[0]**2 + pt[1]**2) >= self.lookahead:
                target = pt
                break

        L = math.sqrt(target[0]**2 + target[1]**2)
        if L < 1e-6:
            steering = 0.0
        else:
            curvature = 2.0 * target[1] / (L * L)
            steering  = math.atan(curvature)
            steering  = max(-0.4, min(0.4, steering))

        speed = self.speed * (1.0 - 0.5 * abs(steering) / 0.4)

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed          = float(speed)
        self.drive_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Visualization
    # -----------------------------------------------------------------------

    def publish_tree(self, V, E):
        ma = MarkerArray()
        m  = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = 'ego_racecar/base_link'
        m.ns = 'tree'; m.id = 0
        m.type   = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.g = 1.0; m.color.a = 0.6
        for (pi, ci) in E:
            p = V[pi]; c = V[ci]
            pt1 = Point(); pt1.x = p.x; pt1.y = p.y
            pt2 = Point(); pt2.x = c.x; pt2.y = c.y
            m.points.append(pt1); m.points.append(pt2)
        ma.markers.append(m)
        self.tree_pub.publish(ma)

    def publish_path(self, path):
        ma = MarkerArray()
        m  = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = 'ego_racecar/base_link'
        m.ns = 'path'; m.id = 0
        m.type   = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.06
        m.color.r = 1.0; m.color.a = 1.0
        for pt in path:
            p = Point(); p.x = pt[0]; p.y = pt[1]; p.z = 0.1
            m.points.append(p)
        ma.markers.append(m)
        self.path_pub.publish(ma)

    def publish_goal(self, goal):
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = 'ego_racecar/base_link'
        m.ns = 'goal'; m.id = 0
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = goal[0]
        m.pose.position.y = goal[1]
        m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3
        m.color.r = 1.0; m.color.g = 1.0; m.color.a = 1.0
        self.goal_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()