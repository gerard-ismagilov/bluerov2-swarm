#!/usr/bin/env python3
"""
USBL Acoustic Navigation Simulator for BlueROV2 Swarm.

Simulates USBL (Ultra-Short Baseline) acoustic positioning system.
In a real deployment this node would be replaced by the driver for
your actual USBL hardware (e.g. Water Linked M64, EvoLogics, LinkQuest).

Publishes position fixes with realistic noise model:
  - Range-dependent noise: sigma increases with distance from transceiver
  - Update rate: 1-5 Hz (typical for USBL systems)
  - Occasional dropout simulation

Topics published (per ROV):
  /rov_N/usbl/fix  (geometry_msgs/PointStamped)  — position in world frame
  /rov_N/usbl/range (std_msgs/Float64)             — slant range to transceiver
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class USBLSimulator(Node):
    """
    Simulates USBL acoustic positioning for a swarm of ROVs.

    The USBL transceiver is assumed to be fixed at the surface/vessel.
    Each ROV has a transponder. The node subscribes to ground-truth
    odometry (from Gazebo) and publishes noisy USBL fixes.
    """

    def __init__(self):
        super().__init__('usbl_simulator')

        # --- Parameters ---
        self.declare_parameter('num_rovs', 4)
        self.declare_parameter('update_rate_hz', 2.0)       # USBL update rate
        self.declare_parameter('base_noise_m', 0.15)        # baseline position noise (m)
        self.declare_parameter('range_noise_factor', 0.01)  # noise grows 1% per meter range
        self.declare_parameter('dropout_probability', 0.02) # 2% chance of dropout per update
        self.declare_parameter('max_range_m', 500.0)        # USBL max operating range
        # Transceiver position (surface vessel / fixed buoy)
        self.declare_parameter('transceiver_x', 0.0)
        self.declare_parameter('transceiver_y', 0.0)
        self.declare_parameter('transceiver_z', 0.0)

        self.num_rovs = self.get_parameter('num_rovs').value
        self.update_rate = self.get_parameter('update_rate_hz').value
        self.base_noise = self.get_parameter('base_noise_m').value
        self.range_noise_factor = self.get_parameter('range_noise_factor').value
        self.dropout_prob = self.get_parameter('dropout_probability').value
        self.max_range = self.get_parameter('max_range_m').value
        tx = self.get_parameter('transceiver_x').value
        ty = self.get_parameter('transceiver_y').value
        tz = self.get_parameter('transceiver_z').value
        self.transceiver_pos = np.array([tx, ty, tz])

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Ground truth positions from Gazebo (set when odom received)
        self.true_positions = {i: None for i in range(self.num_rovs)}

        # Publishers and subscribers per ROV
        self.fix_publishers = {}
        self.range_publishers = {}
        self.odom_subscribers = {}

        for i in range(self.num_rovs):
            ns = f'rov_{i}'
            self.fix_publishers[i] = self.create_publisher(
                PointStamped, f'/{ns}/usbl/fix', 10)
            self.range_publishers[i] = self.create_publisher(
                Float64, f'/{ns}/usbl/range', 10)
            self.odom_subscribers[i] = self.create_subscription(
                Odometry,
                f'/{ns}/odometry',
                lambda msg, idx=i: self._odom_callback(msg, idx),
                sensor_qos
            )

        # Timer for USBL update cycle
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self._publish_usbl_fixes)

        self.get_logger().info(
            f'USBL Simulator started: {self.num_rovs} ROVs, '
            f'{self.update_rate} Hz, base noise {self.base_noise} m'
        )

    def _odom_callback(self, msg: Odometry, rov_idx: int):
        """Store ground truth position from Gazebo odometry."""
        pos = msg.pose.pose.position
        self.true_positions[rov_idx] = np.array([pos.x, pos.y, pos.z])

    def _publish_usbl_fixes(self):
        """Publish noisy USBL position fix for each ROV."""
        now = self.get_clock().now().to_msg()

        for i in range(self.num_rovs):
            true_pos = self.true_positions[i]
            if true_pos is None:
                continue  # ROV not yet visible

            # Compute slant range to transceiver
            delta = true_pos - self.transceiver_pos
            slant_range = float(np.linalg.norm(delta))

            if slant_range > self.max_range:
                self.get_logger().warn(
                    f'ROV {i} out of USBL range: {slant_range:.1f} m > {self.max_range} m',
                    throttle_duration_sec=5.0
                )
                continue

            # Simulate dropout
            if np.random.random() < self.dropout_prob:
                self.get_logger().debug(f'ROV {i}: USBL dropout')
                continue

            # Range-dependent noise model
            # sigma increases with range (multipath, spreading loss)
            noise_sigma = self.base_noise + self.range_noise_factor * slant_range
            noise = np.random.normal(0, noise_sigma, size=3)
            noisy_pos = true_pos + noise

            # Publish position fix
            fix_msg = PointStamped()
            fix_msg.header.stamp = now
            fix_msg.header.frame_id = 'world'
            fix_msg.point.x = float(noisy_pos[0])
            fix_msg.point.y = float(noisy_pos[1])
            fix_msg.point.z = float(noisy_pos[2])
            self.fix_publishers[i].publish(fix_msg)

            # Publish range
            range_msg = Float64()
            range_msg.data = slant_range
            self.range_publishers[i].publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = USBLSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
