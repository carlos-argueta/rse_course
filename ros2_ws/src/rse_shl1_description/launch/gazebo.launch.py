import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get the path to the package's parent directory
    pkg_share = get_package_share_directory('rse_shl1_description')
    
    # Set the GZ_SIM_RESOURCE_PATH environment variable
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, '..'), # Prepend the parent of the share dir
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )
    
    # Launch Ignition Fortress (via ros_gz_sim)
    # Launch Gazebo, passing in a specific world file
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        ])),
        launch_arguments={
            'gz_args': '-r shapes.sdf'
        }.items()
    )

    # Launch RViz + robot_state_publisher + joint GUI from your existing file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rse_shl1_description'), 'launch', 'display.launch.py'
        ])),
    )

    # Spawn the robot using ros_gz_sim/create and the /robot_description topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '2.0',   # Set the X coordinate (e.g., 2.0 meters)
            '-y', '-1.5',  # Set the Y coordinate (e.g., -1.5 meters)
            '-z', '0.5',   # Keep the Z coordinate
        ],
        output='screen',
    )

    return LaunchDescription([
        # --- Add the new action here ---
        gz_resource_path,
        # --- End Add ---
        ign_gazebo,
        description_launch,
        spawn_entity,
    ])