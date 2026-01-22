#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for wheel odometry node.
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('rse_shl1_wheel_odometry')
    config_file = os.path.join(pkg_dir, 'config', 'odometry_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/Isaac) clock if true'
    )
    
    # Wheel odometry node
    wheel_odom_node = Node(
        package='rse_shl1_wheel_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/wheel_odom', '/odom'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        wheel_odom_node,
    ])