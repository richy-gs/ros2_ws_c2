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
    joint_twist_publisher_node = Node(
        package='vanttec_learning',
        executable='joint_twist_publisher_node',
    )

    urdf_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vanttec_learning'),
                'launch',
                'urdf_display.launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        joint_twist_publisher_node,
        urdf_display_launch,
    ])
