"""
Main launch file: 4x BlueROV2 swarm in Gazebo with USBL navigation.

Usage:
  ros2 launch bluerov2_swarm swarm_gazebo.launch.py

  # Override number of ROVs:
  ros2 launch bluerov2_swarm swarm_gazebo.launch.py num_rovs:=2

  # Change formation:
  ros2 launch bluerov2_swarm swarm_gazebo.launch.py formation:=v_shape

Prerequisites:
  - ROS 2 Humble
  - Gazebo Garden (gz-sim 7)
  - ardupilot_gazebo plugin: https://github.com/ArduPilot/ardupilot_gazebo
  - MAVROS 2: sudo apt install ros-humble-mavros ros-humble-mavros-extras
  - ArduSub SITL: installed via ArduPilot

Start ArduSub SITL instances before launching (in separate terminals):
  # ROV 0
  sim_vehicle.py -v ArduSub -f vectored_6dof --model=JSON --out=udp:127.0.0.1:14550 -I0
  # ROV 1
  sim_vehicle.py -v ArduSub -f vectored_6dof --model=JSON --out=udp:127.0.0.1:14560 -I1
  # ROV 2
  sim_vehicle.py -v ArduSub -f vectored_6dof --model=JSON --out=udp:127.0.0.1:14570 -I2
  # ROV 3
  sim_vehicle.py -v ArduSub -f vectored_6dof --model=JSON --out=udp:127.0.0.1:14580 -I3
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_rov_nodes(context, *args, **kwargs):
    """Dynamically generate nodes for each ROV."""
    num_rovs = int(LaunchConfiguration('num_rovs').perform(context))
    formation = LaunchConfiguration('formation').perform(context)
    spacing = LaunchConfiguration('spacing').perform(context)

    pkg = get_package_share_directory('bluerov2_swarm')
    config_file = os.path.join(pkg, 'config', 'controller_params.yaml')

    nodes = []

    # MAVROS base port for ArduSub SITL (each ROV offset by 10)
    mavros_base_port = 14550

    for i in range(num_rovs):
        mavros_port = mavros_base_port + i * 10
        ns = f'rov_{i}'

        # MAVROS node — connects to ArduSub SITL
        mavros_node = Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace=ns,
            parameters=[
                {'fcu_url': f'udp://127.0.0.1:{mavros_port}@14555'},
                {'gcs_url': ''},
                {'target_system_id': i + 1},
                {'target_component_id': 1},
                {'plugin_allowlist': [
                    'sys_status', 'sys_time', 'command',
                    'setpoint_velocity', 'setpoint_position',
                    'imu', 'local_position', 'global_position'
                ]},
            ],
            output='screen',
            remappings=[
                ('/diagnostics', f'/{ns}/diagnostics'),
            ]
        )

        # ROV Bridge (arming + mode management)
        bridge_node = Node(
            package='bluerov2_swarm',
            executable='rov_bridge.py',
            name='rov_bridge',
            namespace=ns,
            parameters=[
                {'rov_id': i},
                {'auto_arm': True},
                {'target_mode': 'GUIDED'},
            ],
            output='screen',
        )

        # Swarm Controller (PID position control)
        controller_node = Node(
            package='bluerov2_swarm',
            executable='swarm_controller.py',
            name='rov_controller',
            namespace=ns,
            parameters=[
                config_file,
                {'rov_id': i},
            ],
            output='screen',
        )

        nodes.extend([mavros_node, bridge_node, controller_node])

    # USBL Simulator (shared — one node handles all ROVs)
    usbl_node = Node(
        package='bluerov2_swarm',
        executable='usbl_simulator.py',
        name='usbl_simulator',
        parameters=[
            {'num_rovs': num_rovs},
            {'update_rate_hz': 2.0},
            {'base_noise_m': 0.15},
            {'range_noise_factor': 0.01},
            {'dropout_probability': 0.02},
            {'transceiver_x': 0.0},
            {'transceiver_y': 0.0},
            {'transceiver_z': 0.5},  # surface
        ],
        output='screen',
    )

    # Formation Manager
    formation_node = Node(
        package='bluerov2_swarm',
        executable='formation_manager.py',
        name='formation_manager',
        parameters=[
            {'num_rovs': num_rovs},
            {'formation_type': formation},
            {'formation_spacing_m': float(spacing)},
        ],
        output='screen',
    )

    nodes.extend([usbl_node, formation_node])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('num_rovs', default_value='4',
                              description='Number of BlueROV2 units in swarm'),
        DeclareLaunchArgument('formation', default_value='line',
                              description='Formation type: line, v_shape, grid, diamond'),
        DeclareLaunchArgument('spacing', default_value='5.0',
                              description='Formation spacing in meters'),
        DeclareLaunchArgument('world', default_value='underwater_world.sdf',
                              description='Gazebo world file'),

        # Gazebo with underwater world
        Node(
            package='ros_gz_sim',
            executable='gzserver',
            name='gazebo',
            arguments=[
                PathJoinSubstitution([
                    FindPackageShare('bluerov2_swarm'), 'worlds',
                    LaunchConfiguration('world')
                ]),
                '--verbose'
            ],
            output='screen',
        ),

        # Gazebo GUI
        Node(
            package='ros_gz_sim',
            executable='gzclient',
            name='gazebo_gui',
            output='screen',
        ),

        # Delay ROV nodes to let Gazebo start
        TimerAction(
            period=5.0,
            actions=[OpaqueFunction(function=generate_rov_nodes)]
        ),
    ])
