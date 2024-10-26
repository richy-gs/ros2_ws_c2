from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_path = get_package_share_path('vanttec_learning')
    robot_model_path = package_path / 'urdf/test_robot.urdf'
    default_rviz_config_path = package_path / 'rviz/urdf_tarea3.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'])
    robot_model_arg = DeclareLaunchArgument(name='test_robot', default_value=str(robot_model_path))
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path))

    model_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'vanttec_learning',
            'urdf_package_path': LaunchConfiguration('test_robot'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ["/j_s"]}],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    return LaunchDescription([
        gui_arg,
        robot_model_arg,
        rviz_arg,
        model_launch,
        joint_state_publisher_node,
    ])

