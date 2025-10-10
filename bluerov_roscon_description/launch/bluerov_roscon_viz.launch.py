import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Define launch arguments
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="bluerov_roscon", description="Name of the robot"
    )

    robot_xacro_arg = DeclareLaunchArgument(
        "robot_xacro",
        default_value=os.path.join(
            FindPackageShare("bluerov_roscon_description").find(
                "bluerov_roscon_description"
            ),
            "urdf",
            "bluerov_roscon.urdf.xacro",
        ),
        description="Path to the robot Xacro file",
    )

    # Command to process the Xacro file
    robot_description_cmd = Command(
        ["xacro ", LaunchConfiguration("robot_xacro"), " robot_namespace:="]
        + [LaunchConfiguration("robot_name")]  # Concatenate as a list
    )

    # Node to publish robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description_cmd, value_type=str)}
        ],
    )

    hw_file = PathJoinSubstitution(
        [
            FindPackageShare("bluerov_roscon_description"),
            "config",
            "hardware_interface.yaml",
        ]
    )

    hw_interface = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[hw_file],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    viz_config = PathJoinSubstitution(
        [FindPackageShare("bluerov_roscon_description"), "config", "view.rviz"]
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz", arguments=["-d", viz_config]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            robot_name_arg,
            robot_xacro_arg,
            robot_state_publisher_node,
            hw_interface,
            joint_state_spawner,
            rviz_node,
        ]
    )
