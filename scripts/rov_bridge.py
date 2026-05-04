#!/usr/bin/env python3
"""
ROV Bridge — ArduSub arming and mode management via MAVROS.

Handles the ArduSub flight controller state machine:
  DISARMED -> GUIDED mode -> ARMED -> accepting setpoints

One instance per ROV. In Gazebo simulation ArduSub SITL must be running
(one instance per ROV on different ports: 5760, 5770, 5780, 5790...).

Connection: MAVROS connects to ArduSub SITL via UDP.

Usage:
  ros2 run bluerov2_swarm rov_bridge.py --ros-args -p rov_id:=0

Services called:
  /rov_N/mavros/cmd/arming
  /rov_N/mavros/set_mode
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class ROVBridge(Node):
    """
    Manages ArduSub state: connects, sets GUIDED mode, arms the vehicle.
    Republishes simplified status for the swarm controller.
    """

    # ArduSub flight modes
    MODE_STABILIZE = 'STABILIZE'
    MODE_GUIDED = 'GUIDED'
    MODE_ALT_HOLD = 'ALT_HOLD'
    MODE_POSHOLD = 'POSHOLD'

    def __init__(self):
        super().__init__('rov_bridge')

        self.declare_parameter('rov_id', 0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('target_mode', 'GUIDED')

        self.rov_id = self.get_parameter('rov_id').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.target_mode = self.get_parameter('target_mode').value

        ns = f'/rov_{self.rov_id}'
        self.ns = ns

        self.cb_group = ReentrantCallbackGroup()

        # Current MAVROS state
        self.mavros_state = None
        self.armed = False
        self.connected = False

        # Subscribers
        self.create_subscription(
            State, f'{ns}/mavros/state',
            self._state_cb, 10,
            callback_group=self.cb_group
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String, f'{ns}/bridge/status', 10)
        self.ready_pub = self.create_publisher(
            Bool, f'{ns}/bridge/ready', 10)

        # Service clients
        self.arm_client = self.create_client(
            CommandBool, f'{ns}/mavros/cmd/arming',
            callback_group=self.cb_group
        )
        self.mode_client = self.create_client(
            SetMode, f'{ns}/mavros/set_mode',
            callback_group=self.cb_group
        )

        # State machine timer — checks and transitions every 2 seconds
        self.sm_timer = self.create_timer(2.0, self._state_machine)

        self.get_logger().info(f'ROV Bridge {self.rov_id} started')

    def _state_cb(self, msg: State):
        self.mavros_state = msg
        self.armed = msg.armed
        self.connected = msg.connected

    def _state_machine(self):
        """
        Periodically drive ArduSub toward armed + GUIDED state.
        Safe to call repeatedly — checks current state before acting.
        """
        if not self.connected:
            self.get_logger().info(
                f'ROV {self.rov_id}: waiting for MAVROS connection...',
                throttle_duration_sec=5.0
            )
            self._publish_status('DISCONNECTED', False)
            return

        if self.mavros_state is None:
            return

        current_mode = self.mavros_state.mode

        # Step 1: set GUIDED mode
        if current_mode != self.target_mode:
            self.get_logger().info(
                f'ROV {self.rov_id}: setting mode {self.target_mode} '
                f'(current: {current_mode})'
            )
            if self.mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = self.target_mode
                future = self.mode_client.call_async(req)
                future.add_done_callback(
                    lambda f: self.get_logger().info(
                        f'ROV {self.rov_id}: set_mode result: '
                        f'{f.result().mode_sent if f.result() else "FAILED"}'
                    )
                )
            self._publish_status(f'SETTING_{self.target_mode}', False)
            return

        # Step 2: arm
        if self.auto_arm and not self.armed:
            self.get_logger().info(f'ROV {self.rov_id}: arming...')
            if self.arm_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                future = self.arm_client.call_async(req)
                future.add_done_callback(
                    lambda f: self.get_logger().info(
                        f'ROV {self.rov_id}: arm result: '
                        f'{f.result().success if f.result() else "FAILED"}'
                    )
                )
            self._publish_status('ARMING', False)
            return

        # All good
        self._publish_status('READY', True)

    def _publish_status(self, status: str, ready: bool):
        msg = String()
        msg.data = f'ROV_{self.rov_id}: {status}'
        self.status_pub.publish(msg)

        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_pub.publish(ready_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ROVBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
