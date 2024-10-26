import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():
    turtle_tf2_publisher_node = Node(
        package='vanttec_learning',
        executable='turtle_tf2_publisher_node',
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )

    robot_state_publisher_node = Node(
        package='vanttec_learning',
        executable='robot_state_publisher_node',
    )

    urdf_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vanttec_learning'),
                'launch',
                'urdf_disp_tarea4.launch.py'
            ])
        ]),
    )

    turtle_follower_node = Node(
        package='vanttec_learning',
        executable='turtle_follower_node',
    )

    return LaunchDescription([
        turtle_tf2_publisher_node,
        turtlesim_node,
        robot_state_publisher_node,
        urdf_display_launch,
        turtle_follower_node,
    ])
