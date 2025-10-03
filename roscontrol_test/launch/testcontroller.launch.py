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
        arguments=[
            "auv_wrench_controller",
            "auv_velocity_controller",
            "auv_pose_controller",
            "--param-file",
            controllers_file,
            "--activate-as-group",
        ],
        output="screen",
    )

    # vel_controllers_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["auv_velocity_controller", "--param-file", controllers_file],
    # )

    # pid_controllers_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["pid_controller", "--param-file", controllers_file],
    # )

    pid_vel_feedback = Node(
        package="roscontrol_test",
        executable="pid_velocity_feedback",
        name="pid_vel_feedback",
        parameters=[
            {
                "dof_state_names": ["auv_vel_x", "auv_vel_z", "auv_vel_yaw"],
            }
        ],
        remappings=[
            ("/pub_pid_measured_topic", "/auv_velocity_controller/measured_state"),
            ("/sub_twist_topic", "/auv/navigator/twist"),
        ],
    )

    nodes = [
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        # vel_controllers_spawner,
        # pid_controllers_spawner,
        pid_vel_feedback,
    ]

    return LaunchDescription(nodes)
