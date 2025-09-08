from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_xacro_arg = DeclareLaunchArgument(
        'robot_xacro',
        default_value=PathJoinSubstitution([FindPackageShare(LaunchConfiguration("robot_description")), 'urdf', 'payload.urdf.xacro']),
        description='Path to the robot Xacro file'
    )

    robot_description_arg = Command(['xacro ',' ', LaunchConfiguration("robot_xacro"), ' robot_namespace:=', LaunchConfiguration('robot_name')])

    return LaunchDescription([
        robot_xacro_arg,
        robot_description_arg,
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=['robot_description', 'controllers.yaml'],
            output='screen',
        ),
        Node(
            package='test_controller_pkg',
            executable='test_node',
            name='test_node',
            output='screen',
        ),
    ])