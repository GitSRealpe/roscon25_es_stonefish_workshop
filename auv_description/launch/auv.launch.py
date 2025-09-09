import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Define launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='auv',
        description='Name of the robot'
    )

    robot_xacro_arg = DeclareLaunchArgument(
        'robot_xacro',
        default_value=os.path.join(
            FindPackageShare('auv_description').find('auv_description'), 
            'urdf', 
            'auv.urdf.xacro'
        ),
        description='Path to the robot Xacro file'
    )

    # Command to process the Xacro file
    robot_description_cmd = Command(
        [
            'xacro ', 
            LaunchConfiguration('robot_xacro'), 
            ' robot_namespace:='
        ] + [LaunchConfiguration('robot_name')]  # Concatenate as a list
    )

    # Node to publish robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description_cmd, value_type=str
            )
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        robot_xacro_arg,
        robot_state_publisher_node,
    ])
