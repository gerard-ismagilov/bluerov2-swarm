#!/usr/bin/env python3
"""
Swarm Controller for BlueROV2.

One instance runs per ROV (use namespace parameter).
Implements a 6-DOF PID position controller that:
  1. Takes target pose from formation_manager
  2. Estimates current position from USBL fix + onboard IMU fusion
  3. Computes velocity commands
  4. Publishes to MAVROS setpoint topics for ArduSub

Control architecture:
  USBL fix + IMU -> EKF position estimate
  -> PID (x, y, z, yaw) -> body velocity setpoint
  -> MAVROS /setpoint_velocity/cmd_vel

Subscribed topics (in ROV namespace):
  ~/usbl/fix         (geometry_msgs/PointStamped)
  ~/target_pose      (geometry_msgs/PoseStamped)
  ~/odometry         (nav_msgs/Odometry)  — from Gazebo / ArduSub EKF

Published topics:
  ~/mavros/setpoint_velocity/cmd_vel  (geometry_msgs/TwistStamped)
  ~/mavros/set_mode                   (mavros_msgs/SetMode)
  ~/controller/status                 (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


class PIDController:
    """Simple PID with anti-windup and output clamping."""

    def __init__(self, kp, ki, kd, output_limit=1.0, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self._integral += error * dt
        self._integral = np.clip(
            self._integral, -self.integral_limit, self.integral_limit)
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return float(np.clip(output, -self.output_limit, self.output_limit))


class SimpleEKF:
    """
    Minimal EKF to fuse USBL (slow, noisy) with odometry (fast, drifts).
    State: [x, y, z]
    """

    def __init__(self, process_noise=0.01, usbl_noise=0.2):
        self.x = np.zeros(3)
        self.P = np.eye(3) * 10.0   # initial uncertainty
        self.Q = np.eye(3) * process_noise
        self.R = np.eye(3) * usbl_noise ** 2
        self.initialized = False

    def predict(self, dx: np.ndarray):
        """Propagate state with velocity step."""
        self.x += dx
        self.P += self.Q

    def update_usbl(self, z: np.ndarray):
        """Incorporate USBL measurement."""
        if not self.initialized:
            self.x = z.copy()
            self.initialized = True
            return
        H = np.eye(3)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(3) - K @ H) @ self.P


class ROVController(Node):
    """Position controller for a single BlueROV2."""

    def __init__(self):
        super().__init__('rov_controller')

        # Parameters
        self.declare_parameter('rov_id', 0)
        self.declare_parameter('max_horizontal_vel', 1.0)   # m/s
        self.declare_parameter('max_vertical_vel', 0.5)     # m/s
        self.declare_parameter('max_yaw_rate', 0.5)         # rad/s
        self.declare_parameter('position_tolerance', 0.3)   # m — considered "at target"
        self.declare_parameter('depth_hold', -3.0)          # default depth (negative = below surface)
        # PID gains — tune these for your ROV
        self.declare_parameter('pid_xy_kp', 0.6)
        self.declare_parameter('pid_xy_ki', 0.02)
        self.declare_parameter('pid_xy_kd', 0.15)
        self.declare_parameter('pid_z_kp', 0.8)
        self.declare_parameter('pid_z_ki', 0.05)
        self.declare_parameter('pid_z_kd', 0.1)
        self.declare_parameter('pid_yaw_kp', 1.2)
        self.declare_parameter('pid_yaw_ki', 0.0)
        self.declare_parameter('pid_yaw_kd', 0.2)
        self.declare_parameter('control_rate_hz', 20.0)

        self.rov_id = self.get_parameter('rov_id').value
        self.max_hvel = self.get_parameter('max_horizontal_vel').value
        self.max_vvel = self.get_parameter('max_vertical_vel').value
        self.max_yaw = self.get_parameter('max_yaw_rate').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        rate = self.get_parameter('control_rate_hz').value

        # PID controllers
        self.pid_x = PIDController(
            self.get_parameter('pid_xy_kp').value,
            self.get_parameter('pid_xy_ki').value,
            self.get_parameter('pid_xy_kd').value,
            output_limit=self.max_hvel
        )
        self.pid_y = PIDController(
            self.get_parameter('pid_xy_kp').value,
            self.get_parameter('pid_xy_ki').value,
            self.get_parameter('pid_xy_kd').value,
            output_limit=self.max_hvel
        )
        self.pid_z = PIDController(
            self.get_parameter('pid_z_kp').value,
            self.get_parameter('pid_z_ki').value,
            self.get_parameter('pid_z_kd').value,
            output_limit=self.max_vvel
        )
        self.pid_yaw = PIDController(
            self.get_parameter('pid_yaw_kp').value,
            self.get_parameter('pid_yaw_ki').value,
            self.get_parameter('pid_yaw_kd').value,
            output_limit=self.max_yaw
        )

        self.ekf = SimpleEKF()
        self.current_yaw = 0.0
        self.target_pose = None
        self.last_time = self.get_clock().now()
        self.at_target = False

        ns = f'/rov_{self.rov_id}'

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, f'{ns}/mavros/setpoint_velocity/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, f'{ns}/controller/status', 10)
        self.at_target_pub = self.create_publisher(
            Bool, f'{ns}/controller/at_target', 10)

        # Subscribers
        self.create_subscription(
            PointStamped, f'{ns}/usbl/fix', self._usbl_cb, 10)
        self.create_subscription(
            PoseStamped, f'{ns}/target_pose', self._target_cb, 10)
        self.create_subscription(
            Odometry, f'{ns}/odometry', self._odom_cb, 10)

        # Control loop timer
        self.timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'ROV Controller {self.rov_id} started | '
            f'max_hvel={self.max_hvel} m/s | rate={rate} Hz'
        )

    def _usbl_cb(self, msg: PointStamped):
        z = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.ekf.update_usbl(z)

    def _target_cb(self, msg: PoseStamped):
        self.target_pose = msg

    def _odom_cb(self, msg: Odometry):
        """Use odometry for EKF prediction step and yaw extraction."""
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

        # If EKF not yet initialized by USBL, bootstrap from odometry
        if not self.ekf.initialized:
            p = msg.pose.pose.position
            self.ekf.x = np.array([p.x, p.y, p.z])
            self.ekf.initialized = True

    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return float((angle + np.pi) % (2 * np.pi) - np.pi)

    def _control_loop(self):
        """Main PID control loop — runs at control_rate_hz."""
        if not self.ekf.initialized or self.target_pose is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0 or dt > 1.0:
            return

        pos = self.ekf.x
        tgt = self.target_pose.pose.position

        # Position errors in world frame
        err_x = tgt.x - pos[0]
        err_y = tgt.y - pos[1]
        err_z = tgt.z - pos[2]

        # Yaw error: compute desired yaw toward target (x-y plane)
        desired_yaw = np.arctan2(err_y, err_x) if (abs(err_x) + abs(err_y)) > 0.2 else self.current_yaw
        q_tgt = self.target_pose.pose.orientation
        siny = 2.0 * (q_tgt.w * q_tgt.z + q_tgt.x * q_tgt.y)
        cosy = 1.0 - 2.0 * (q_tgt.y * q_tgt.y + q_tgt.z * q_tgt.z)
        desired_yaw = np.arctan2(siny, cosy)
        err_yaw = self._wrap_angle(desired_yaw - self.current_yaw)

        # Check if at target
        dist = np.sqrt(err_x**2 + err_y**2 + err_z**2)
        self.at_target = dist < self.pos_tol

        # Rotate error from world frame to body frame
        cos_y = np.cos(self.current_yaw)
        sin_y = np.sin(self.current_yaw)
        err_body_x =  err_x * cos_y + err_y * sin_y
        err_body_y = -err_x * sin_y + err_y * cos_y

        # PID outputs (body frame)
        vx = self.pid_x.compute(err_body_x, dt)
        vy = self.pid_y.compute(err_body_y, dt)
        vz = self.pid_z.compute(err_z, dt)
        wz = self.pid_yaw.compute(err_yaw, dt)

        # Publish velocity command to MAVROS
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = f'rov_{self.rov_id}/base_link'
        cmd.twist.linear.x = float(np.clip(vx, -self.max_hvel, self.max_hvel))
        cmd.twist.linear.y = float(np.clip(vy, -self.max_hvel, self.max_hvel))
        cmd.twist.linear.z = float(np.clip(vz, -self.max_vvel, self.max_vvel))
        cmd.twist.angular.z = float(np.clip(wz, -self.max_yaw, self.max_yaw))
        self.cmd_vel_pub.publish(cmd)

        # Status
        status = String()
        status.data = (
            f'ROV_{self.rov_id} | '
            f'pos=[{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}] | '
            f'err={dist:.2f}m | '
            f'{"AT_TARGET" if self.at_target else "MOVING"}'
        )
        self.status_pub.publish(status)

        at_target_msg = Bool()
        at_target_msg.data = self.at_target
        self.at_target_pub.publish(at_target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ROVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
