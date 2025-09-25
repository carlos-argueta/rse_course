from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import xacro
import os

def generate_launch_description():

    # Get the path to your .xacro file
    xacro_file = os.path.join(get_package_share_directory('rse_shl1_description'), 'xacro', 'shl1_robot.xacro')

    # Process the xacro file to get the URDF XML as a string
    robot_description_config = xacro.process_file(xacro_file)

    #with open(urdf_file_path, 'r') as urdf_file:
    #    robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': True}],
    )
    
    # ADDED: This node provides a GUI to control the robot's joints
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("rse_shl1_description"), 'rviz', "display.rviz"]),],
        parameters=[{'use_sim_time': True}]
    )
    
    # ADDED: The new node to the launch description
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])