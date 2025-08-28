from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ------------------------------
    # Paths to packages & launch files
    # ------------------------------
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    # robot_nav_pkg = get_package_share_directory('robot_nav_utils')

    # ------------------------------
    # Launch TurtleBot3 Gazebo world
    # ------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # ------------------------------
    # Move-to-home node (after Nav2 starts)
    # ------------------------------
    move_to_home_node = TimerAction(
        period=5.0,  # wait 5 seconds to let Nav2 initialize
        actions=[
            Node(
                package='robot_nav_utils',
                executable='move',  # entry point in setup.py
                name='move_to_home_service',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        # move_to_home_node
    ])
