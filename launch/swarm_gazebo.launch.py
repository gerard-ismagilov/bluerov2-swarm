from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction,
    OpaqueFunction, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_rov_nodes(context, *args, **kwargs):
    num_rovs = int(LaunchConfiguration('num_rovs').perform(context))
    formation = LaunchConfiguration('formation').perform(context)
    spacing = LaunchConfiguration('spacing').perform(context)
    pkg = get_package_share_directory('bluerov2_swarm')
    config = os.path.join(pkg, 'config', 'controller_params.yaml')

    nodes = []
    for i in range(num_rovs):
        nodes.append(Node(
            package='bluerov2_swarm',
            executable='swarm_controller.py',
            name=f'rov_controller_{i}',
            parameters=[config, {'rov_id': i}],
            output='screen',
        ))

    nodes.append(Node(
        package='bluerov2_swarm',
        executable='usbl_simulator.py',
        name='usbl_simulator',
        parameters=[{'num_rovs': num_rovs}],
        output='screen',
    ))

    nodes.append(Node(
        package='bluerov2_swarm',
        executable='formation_manager.py',
        name='formation_manager',
        parameters=[{
            'num_rovs': num_rovs,
            'formation_type': formation,
            'formation_spacing_m': float(spacing),
        }],
        output='screen',
    ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_rovs', default_value='4'),
        DeclareLaunchArgument('formation', default_value='line'),
        DeclareLaunchArgument('spacing', default_value='5.0'),

        ExecuteProcess(
            cmd=[
                'gzserver',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                '--verbose'
            ],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
        ),

        TimerAction(
            period=5.0,
            actions=[OpaqueFunction(function=generate_rov_nodes)]
        ),
    ])
