#!/usr/bin/env python3
import math
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from sensor_msgs.msg import LaserScan
import sys; sys.path.insert(0, "/sim_ws/src/mpc/scripts"); from utils import nearest_point

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

@dataclass
class mpc_config:
    NXK: int = 4
    NU: int = 2
    TK: int = 8

    Rk: list = field(default_factory=lambda: np.diag([10.0, 10.0]))
    Rdk: list = field(default_factory=lambda: np.diag([10.0, 10.0]))
    Qk: list = field(default_factory=lambda: np.diag([13.5, 13.5, 13.0, 13.0]))
    Qfk: list = field(default_factory=lambda: np.diag([13.5, 13.5, 13.0, 13.0]))

    N_IND_SEARCH: int = 20
    DTK: float = 0.1
    dlk: float = 0.1
    LENGTH: float = 0.58
    WIDTH: float = 0.31
    WB: float = 0.33
    MIN_STEER: float = -0.4189
    MAX_STEER: float = 0.4189
    MAX_DSTEER: float = np.deg2rad(180.0)
    MAX_SPEED: float = 2.0
    MIN_SPEED: float = 0.0
    MAX_ACCEL: float = 1.0


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    v: float = 0.0
    yaw: float = 0.0


class MPC(Node):
    def __init__(self):
        super().__init__('mpc_node')

        # 订阅 odometry，发布 drive
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # 加载 waypoints: x, y, v, yaw
        waypoints = np.loadtxt(
            '/sim_ws/src/mpc/scripts/levine_waypoints_mpc.csv', delimiter=',')
        self.ref_x   = waypoints[:, 0]
        self.ref_y   = waypoints[:, 1]
        self.ref_v   = waypoints[:, 2]
        self.ref_yaw = waypoints[:, 3]
        self.prev_yaw = 0.0
        self.yaw_offset = 0.0
        self.config = mpc_config()
        self.odelta = None
        self.oa = None
        self.init_flag = 0

        self.mpc_prob_init()
        self.get_logger().info('MPC Node Started')
        self.ref_path_pub = self.create_publisher(Marker, '/ref_path', 10)
        self.pred_path_pub = self.create_publisher(Marker, '/pred_path', 10)

    def pose_callback(self, pose_msg):
        # 提取当前状态
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        q = pose_msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        v = pose_msg.twist.twist.linear.x
        vehicle_state = State(x=x, y=y, v=v, yaw=yaw)

        # 计算参考轨迹
        ref_path = self.calc_ref_trajectory(
            vehicle_state, self.ref_x, self.ref_y, self.ref_yaw, self.ref_v)
        x0 = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]
        print(f"x:{vehicle_state.x:.2f} y:{vehicle_state.y:.2f} yaw:{vehicle_state.yaw:.2f} ref_yaw0:{ref_path[3,0]:.2f} ref_yaw_end:{ref_path[3,-1]:.2f} steer:{self.odelta[0] if self.odelta is not None else 0:.3f}")
        # 求解 MPC
        (self.oa, self.odelta, ox, oy, oyaw, ov, state_predict) = \
            self.linear_mpc_control(ref_path, x0, self.oa, self.odelta)

        if self.odelta is None or self.oa is None:
            return

        # 发布控制指令
        steer_output = self.odelta[0]
        speed_output = vehicle_state.v + self.oa[0] * self.config.DTK
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer_output
        drive_msg.drive.speed = speed_output
        self.drive_pub.publish(drive_msg)
        self.publish_markers(ref_path, state_predict)

    def mpc_prob_init(self):
        self.xk = cvxpy.Variable((self.config.NXK, self.config.TK + 1))
        self.uk = cvxpy.Variable((self.config.NU, self.config.TK))
        objective = 0.0
        constraints = []

        self.x0k = cvxpy.Parameter((self.config.NXK,))
        self.x0k.value = np.zeros((self.config.NXK,))

        self.ref_traj_k = cvxpy.Parameter((self.config.NXK, self.config.TK + 1))
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))

        R_block = block_diag(tuple([self.config.Rk] * self.config.TK))
        Rd_block = block_diag(tuple([self.config.Rdk] * (self.config.TK - 1)))
        Q_block = [self.config.Qk] * (self.config.TK)
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))

        # -------------------------------------------------------
        # 目标函数 Part 1: 控制输入代价 u^T R u
        objective += cvxpy.quad_form(
            cvxpy.vec(self.uk), csc_matrix(R_block))

        # 目标函数 Part 2: 状态偏差代价 (x-xref)^T Q (x-xref)
        error = self.xk - self.ref_traj_k
        objective += cvxpy.quad_form(
            cvxpy.vec(error), csc_matrix(Q_block))

        # 目标函数 Part 3: 控制变化率代价 (u_{t+1}-u_t)^T Rd (u_{t+1}-u_t)
        delta_u = self.uk[:, 1:] - self.uk[:, :-1]
        objective += cvxpy.quad_form(
            cvxpy.vec(delta_u), csc_matrix(Rd_block))
        # -------------------------------------------------------

        # 构建 A, B, C 稀疏矩阵（初始化为零轨迹）
        A_block = []
        B_block = []
        C_block = []
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        m, n = A_block.shape
        self.Annz_k = cvxpy.Parameter(A_block.nnz)
        data = np.ones(self.Annz_k.size)
        rows = A_block.row * n + A_block.col
        cols = np.arange(self.Annz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Annz_k.size))
        self.Annz_k.value = A_block.data
        self.Ak_ = cvxpy.reshape(Indexer @ self.Annz_k, (m, n), order="C")

        m, n = B_block.shape
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)
        data = np.ones(self.Bnnz_k.size)
        rows = B_block.row * n + B_block.col
        cols = np.arange(self.Bnnz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Bnnz_k.size))
        self.Bk_ = cvxpy.reshape(Indexer @ self.Bnnz_k, (m, n), order="C")
        self.Bnnz_k.value = B_block.data

        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        # -------------------------------------------------------
        # 约束 Part 1: 动力学约束
        # xk[:,t+1] = Ak_ @ xk[:,t] + Bk_ @ uk[:,t] + Ck_[t*NXK:(t+1)*NXK]
        # 用 block 形式：X_1:T = Ak_ @ X_0:T-1 + Bk_ @ U + Ck_
        constraints += [
            cvxpy.vec(self.xk[:, 1:]) ==
            self.Ak_ @ cvxpy.vec(self.xk[:, :-1]) +
            self.Bk_ @ cvxpy.vec(self.uk) +
            self.Ck_
        ]

        # 约束 Part 2: 转向变化率约束
        constraints += [
            cvxpy.abs(self.uk[1, 1:] - self.uk[1, :-1]) <=
            self.config.MAX_DSTEER * self.config.DTK
        ]

        # 约束 Part 3: 状态和输入上下界 + 初始状态
        constraints += [self.xk[:, 0] == self.x0k]
        constraints += [self.xk[2, :] <= self.config.MAX_SPEED]
        constraints += [self.xk[2, :] >= self.config.MIN_SPEED]
        constraints += [self.uk[0, :] <= self.config.MAX_ACCEL]
        constraints += [self.uk[0, :] >= -self.config.MAX_ACCEL]
        constraints += [self.uk[1, :] <= self.config.MAX_STEER]
        constraints += [self.uk[1, :] >= self.config.MIN_STEER]
        # -------------------------------------------------------

        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp):
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ncourse = len(cx)
        cyaw = cyaw.copy()
        _, _, _, ind = nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)
        
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        travel = abs(state.v) * self.config.DTK
        dind = max(travel / self.config.dlk, 1.0)
        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.config.TK)), 0, 0).astype(int)
        ind_list[ind_list >= ncourse] -= ncourse
        print(f"ind:{ind} ind_list:{ind_list[0]}~{ind_list[-1]} ncourse:{ncourse}")
        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
        # 先取出horizon内的yaw，再做连续性处理
        raw_yaws = cyaw[ind_list].copy()
        # 对齐到车辆yaw（考虑unwrap后可能差很多圈）
        offset = state.yaw - raw_yaws[0]
        n_turns = round(offset / (2 * math.pi))
        raw_yaws += n_turns * 2 * math.pi
        # horizon内连续
        for i in range(1, len(raw_yaws)):
            diff = raw_yaws[i] - raw_yaws[i-1]
            if diff > math.pi:
                raw_yaws[i] -= 2 * math.pi
            elif diff < -math.pi:
                raw_yaws[i] += 2 * math.pi
        ref_traj[3, :] = raw_yaws       


        return ref_traj

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]
        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw
        return path_predict

    def update_state(self, state, a, delta):
        if delta >= self.config.MAX_STEER:
            delta = self.config.MAX_STEER
        elif delta <= -self.config.MAX_STEER:
            delta = -self.config.MAX_STEER
        state.x = state.x + state.v * math.cos(state.yaw) * self.config.DTK
        state.y = state.y + state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw = state.yaw + (state.v / self.config.WB) * math.tan(delta) * self.config.DTK
        state.v = state.v + a * self.config.DTK
        if state.v > self.config.MAX_SPEED:
            state.v = self.config.MAX_SPEED
        elif state.v < self.config.MIN_SPEED:
            state.v = self.config.MIN_SPEED
        return state

    def get_model_matrix(self, v, phi, delta):
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block
        self.ref_traj_k.value = ref_traj

        self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        if (self.MPC_prob.status == cvxpy.OPTIMAL or
                self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()
        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def linear_mpc_control(self, ref_path, x0, oa, od):
        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        path_predict = self.predict_motion(x0, oa, od, ref_path)
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.mpc_prob_solve(
            ref_path, path_predict, x0)

        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, path_predict

    def publish_markers(self, ref_path, pred_path):
        # 参考轨迹（绿色）
        ref_marker = Marker()
        ref_marker.header.frame_id = 'map'
        ref_marker.header.stamp = self.get_clock().now().to_msg()
        ref_marker.type = Marker.LINE_STRIP
        ref_marker.action = Marker.ADD
        ref_marker.id = 0
        ref_marker.scale.x = 0.05
        ref_marker.color.r = 0.0
        ref_marker.color.g = 1.0
        ref_marker.color.b = 0.0
        ref_marker.color.a = 1.0
        for i in range(ref_path.shape[1]):
            p = Point()
            p.x = ref_path[0, i]
            p.y = ref_path[1, i]
            p.z = 0.0
            ref_marker.points.append(p)
        self.ref_path_pub.publish(ref_marker)

        # 预测轨迹（蓝色）
        pred_marker = Marker()
        pred_marker.header.frame_id = 'map'
        pred_marker.header.stamp = self.get_clock().now().to_msg()
        pred_marker.type = Marker.LINE_STRIP
        pred_marker.action = Marker.ADD
        pred_marker.id = 1
        pred_marker.scale.x = 0.05
        pred_marker.color.r = 0.0
        pred_marker.color.g = 0.0
        pred_marker.color.b = 1.0
        pred_marker.color.a = 1.0
        for i in range(pred_path.shape[1]):
            p = Point()
            p.x = pred_path[0, i]
            p.y = pred_path[1, i]
            p.z = 0.0
            pred_marker.points.append(p)
        self.pred_path_pub.publish(pred_marker)

def main(args=None):
    rclpy.init(args=args)
    print("MPC Initialized")
    mpc_node = MPC()
    rclpy.spin(mpc_node)
    mpc_node.destroy_node()
    rclpy.shutdown()

main()
