#!/usr/bin/env python3
"""
Formation Manager for BlueROV2 Swarm.

Manages swarm formation geometry and assigns target waypoints to each ROV.
Supports multiple formation types suitable for seabed mining operations.

Formation types:
  - LINE:     ROVs in a line, good for strip coverage
  - V_SHAPE:  V-formation, good for area scanning
  - GRID:     Grid pattern for dense coverage
  - DIAMOND:  Diamond formation, good for obstacle avoidance
  - ADAPTIVE: Dynamically adjusts spacing based on USBL range data

Published topics:
  /swarm/formation_mode  (std_msgs/String)
  /rov_N/target_pose     (geometry_msgs/PoseStamped)  — goal for each ROV
"""

import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class FormationType(Enum):
    LINE = 'line'
    V_SHAPE = 'v_shape'
    GRID = 'grid'
    DIAMOND = 'diamond'
    ADAPTIVE = 'adaptive'


class FormationManager(Node):
    """
    Computes and publishes target poses for each ROV in the swarm.

    The leader ROV (index 0) follows the mission waypoints.
    Follower ROVs maintain their formation offset relative to the leader.
    """

    # Formation offset templates (x_forward, y_lateral, z_depth) in meters
    # Indexed by num_rovs -> list of (dx, dy, dz) offsets per ROV
    FORMATIONS = {
        FormationType.LINE: {
            # All ROVs in a line abreast (side by side)
            # Good for strip mining — covers maximum width
            'offsets': lambda n, spacing: [
                (0.0, (i - (n-1)/2.0) * spacing, 0.0)
                for i in range(n)
            ]
        },
        FormationType.V_SHAPE: {
            # V-shape: leader at front, followers trail behind and spread out
            # Good for forward area scanning
            'offsets': lambda n, spacing: [
                (-abs(i - (n-1)/2.0) * spacing * 0.7,
                 (i - (n-1)/2.0) * spacing,
                 0.0)
                for i in range(n)
            ]
        },
        FormationType.DIAMOND: {
            # Diamond: works best for 4 ROVs
            'offsets': lambda n, spacing: [
                (0, 0, 0),
                (-spacing, -spacing, 0),
                (-spacing,  spacing, 0),
                (-2*spacing, 0, 0),
            ][:n]
        },
        FormationType.GRID: {
            # Grid: sqrt(n) x sqrt(n) arrangement
            'offsets': lambda n, spacing: [
                ((i // int(np.ceil(np.sqrt(n)))) * spacing,
                 (i  % int(np.ceil(np.sqrt(n)))) * spacing - spacing,
                 0.0)
                for i in range(n)
            ]
        },
    }

    def __init__(self):
        super().__init__('formation_manager')

        self.declare_parameter('num_rovs', 4)
        self.declare_parameter('formation_type', 'line')
        self.declare_parameter('formation_spacing_m', 5.0)   # meters between ROVs
        self.declare_parameter('update_rate_hz', 10.0)

        self.num_rovs = self.get_parameter('num_rovs').value
        formation_str = self.get_parameter('formation_type').value
        self.spacing = self.get_parameter('formation_spacing_m').value
        rate = self.get_parameter('update_rate_hz').value

        try:
            self.formation = FormationType(formation_str)
        except ValueError:
            self.get_logger().warn(
                f'Unknown formation "{formation_str}", defaulting to LINE')
            self.formation = FormationType.LINE

        # Leader position (ROV 0)
        self.leader_pose = None
        self.leader_heading = 0.0  # radians, yaw

        # Current positions from USBL
        self.usbl_positions = {i: None for i in range(self.num_rovs)}

        # Target pose publishers
        self.target_publishers = {}
        for i in range(self.num_rovs):
            self.target_publishers[i] = self.create_publisher(
                PoseStamped, f'/rov_{i}/target_pose', 10)

        # Formation mode publisher
        self.mode_pub = self.create_publisher(String, '/swarm/formation_mode', 10)

        # Subscribe to leader odometry
        self.create_subscription(
            Odometry, '/rov_0/odometry', self._leader_odom_cb, 10)

        # Subscribe to USBL fixes for all ROVs
        for i in range(self.num_rovs):
            self.create_subscription(
                PointStamped,
                f'/rov_{i}/usbl/fix',
                lambda msg, idx=i: self._usbl_cb(msg, idx),
                10
            )

        # Subscribe to formation change commands
        self.create_subscription(
            String, '/swarm/set_formation', self._formation_cmd_cb, 10)

        self.timer = self.create_timer(1.0 / rate, self._publish_targets)

        self.get_logger().info(
            f'Formation Manager: {self.num_rovs} ROVs, '
            f'formation={self.formation.value}, spacing={self.spacing}m'
        )

    def _leader_odom_cb(self, msg: Odometry):
        self.leader_pose = msg.pose.pose
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.leader_heading = np.arctan2(siny_cosp, cosy_cosp)

    def _usbl_cb(self, msg: PointStamped, rov_idx: int):
        self.usbl_positions[rov_idx] = np.array([
            msg.point.x, msg.point.y, msg.point.z
        ])

    def _formation_cmd_cb(self, msg: String):
        try:
            new_formation = FormationType(msg.data.lower())
            self.formation = new_formation
            self.get_logger().info(f'Formation changed to: {new_formation.value}')
        except ValueError:
            self.get_logger().warn(f'Unknown formation type: {msg.data}')

    def _get_offsets(self):
        """Compute formation offsets for current formation type."""
        if self.formation in self.FORMATIONS:
            return self.FORMATIONS[self.formation]['offsets'](
                self.num_rovs, self.spacing)
        return [(0, 0, 0)] * self.num_rovs

    def _rotate_offset(self, dx, dy, heading):
        """Rotate formation offset to align with leader heading."""
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        rx = dx * cos_h - dy * sin_h
        ry = dx * sin_h + dy * cos_h
        return rx, ry

    def _publish_targets(self):
        if self.leader_pose is None:
            return

        now = self.get_clock().now().to_msg()
        offsets = self._get_offsets()

        lx = self.leader_pose.position.x
        ly = self.leader_pose.position.y
        lz = self.leader_pose.position.z

        for i, (dx, dy, dz) in enumerate(offsets):
            # Rotate offset to world frame based on leader heading
            rx, ry = self._rotate_offset(dx, dy, self.leader_heading)

            target = PoseStamped()
            target.header.stamp = now
            target.header.frame_id = 'world'
            target.pose.position.x = lx + rx
            target.pose.position.y = ly + ry
            target.pose.position.z = lz + dz
            # All ROVs face the same direction as leader
            target.pose.orientation = self.leader_pose.orientation
            self.target_publishers[i].publish(target)

        # Publish current mode
        mode_msg = String()
        mode_msg.data = self.formation.value
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FormationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
