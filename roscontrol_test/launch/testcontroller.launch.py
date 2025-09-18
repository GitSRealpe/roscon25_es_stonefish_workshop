from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("auv_description"), "urdf", "auv.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("roscontrol_test"), "config", "controller.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],  # YAML with your controller definition
        output="screen",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # Spawn your custom controller (name must match the one in controllers.yaml)
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["auv_wrench_controller", "--param-file", controllers_file],
        output="screen",
    )

    # pid_controllers_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["pid_controller", "--param-file", controllers_file],
    # )

    nodes = [
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        # pid_controllers_spawner,
    ]

    return LaunchDescription(nodes)
