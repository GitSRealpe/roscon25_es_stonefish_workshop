import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='auv',  # Edit for other robots
        description='Name of the robot'
    )

    robot_package_arg = DeclareLaunchArgument(
        'robot_package',
        default_value=['cola2_',LaunchConfiguration("robot_name")],
        description='Package of cola2 for the robot'
    )

    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=[LaunchConfiguration("robot_name"),'_description'],
        description='Description of the robot'
    )

    period_controller_arg = DeclareLaunchArgument(
        'period_controller',
        default_value="0.1",
        description='Period for the controller'
    )

    n_thrusters_arg = DeclareLaunchArgument(
        'n_thrusters_controller',
        default_value="3",    # Edit for other robots
        description='N_thrusters for the controller'
    )

    robot_xacro_arg = DeclareLaunchArgument(
        'robot_xacro',
        default_value=PathJoinSubstitution([FindPackageShare(LaunchConfiguration("robot_description")), 'urdf', 'payload.urdf.xacro']),
        description='Path to the robot Xacro file'
    )

    robot_description = Command(['xacro ',' ', LaunchConfiguration("robot_xacro"), ' robot_namespace:=', LaunchConfiguration('robot_name')])


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description}],
        namespace = LaunchConfiguration('robot_name')
    )

    diagnostic_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[[LaunchConfiguration('config_folder'), '/', 'diagnostic_aggregator.yaml']],
    )

    config_folder_arg = DeclareLaunchArgument(
        'config_folder',
        default_value=PathJoinSubstitution([FindPackageShare(LaunchConfiguration("robot_package")), 'sim_config']),
        description='Path to the config folder'
    )

    captain_node = Node(
        package='cola2_control',
        executable='captain_node',
        name='captain',
        output='screen',
        parameters=[[LaunchConfiguration('config_folder'), '/', 'captain.yaml']], # Edit captain file to match robot
    )

    pilot_node = Node(
        package='cola2_control',
        executable='pilot_node',
        name='pilot',
        output='screen',
        parameters=[[LaunchConfiguration('config_folder'), '/', 'pilot.yaml']],
    )

    controller_node = Node(
        package='cola2_control',
        executable='controller_node',
        name='controller',
        output='screen',
        parameters=[[LaunchConfiguration('config_folder'), '/', 'controller.yaml']],
        # arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        arguments=[
            '--period', LaunchConfiguration("period_controller"),
            '--n_thrusters', LaunchConfiguration("n_thrusters_controller"),
            "--robot_name", LaunchConfiguration("robot_name"),
        ]
    )

    teleoperation_node = Node(
        package='cola2_control',
        executable='teleoperation_node',
        name='teleoperation',
        output='screen',
        parameters=[[LaunchConfiguration('config_folder'), '/', 'teleoperation.yaml']],
    )

    navigator_node = Node(
        package='cola2_nav',
        executable='navigator_node',
        name='navigator',
        output='screen',
        parameters=[[LaunchConfiguration('config_folder'), '/', 'navigator.yaml']],#[LaunchConfiguration('navigator_path')],
    )

    odom_tf_node = Node(
        package='auv_description',
        executable='odom_to_world_ned',
        name='odom_tf',
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        respawn=True, 
        parameters=[{
            'dev': '/dev/input/js0',
            'dev_ff': '',
            'deadzone': 0.05
        }],
        output='screen'
    )

    logitech_node = Node(
        package='cola2_control',
        executable='logitech_fx10_to_teleoperation_node.py',
        name='logitech_fx10_to_teleoperation',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare(LaunchConfiguration("robot_package")), 'config', 'logitech_fx10_to_teleoperation.yaml'])],
    )

    messages_node = GroupAction([
        Node(package='topic_tools', executable='throttle', name='navigation_throttle', arguments=['messages', 'navigator/navigation', '4.0']),
        Node(package='topic_tools', executable='throttle', name='odometry_throttle', arguments=['messages', 'navigator/odometry', '4.0']),
        Node(package='topic_tools', executable='throttle', name='merged_world_waypoint_req_throttle', arguments=['messages', 'controller/merged_world_waypoint_req', '2.0']),
        Node(package='topic_tools', executable='throttle', name='merged_body_velocity_req_throttle', arguments=['messages', 'controller/merged_body_velocity_req', '2.0']),
        Node(package='topic_tools', executable='throttle', name='merged_body_force_req_throttle', arguments=['messages', 'controller/merged_body_force_req', '2.0']),
        Node(package='topic_tools', executable='throttle', name='thruster_setpoints_throttle', arguments=['messages', 'setpoints_selector/thruster_setpoints', '4.0']),
    ])

    gui_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen'
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world2ned',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '3.1415', '--frame-id', 'world', '--child-frame-id', 'world_ned'],
        output='screen'
    )

    ros2_stonefish_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stonefish_ros2'),
                'launch',
                'stonefish_simulator.launch.py'
            ])
        ]),
        launch_arguments = {
            'simulation_data' : PathJoinSubstitution([FindPackageShare('auv_stonefish'), 'resources']),
            'scenario_desc' : PathJoinSubstitution([FindPackageShare('auv_stonefish'), 'scenarios', 'sea.scn']),  # Edit for other robots
            'simulation_rate' : '300.0',
            'window_res_x' : '1440',
            'window_res_y' : '900',
            'rendering_quality' : 'low'
        }.items()
    )

    translator_node = Node(package='auv_stonefish', executable='auv_stonefish_node', name='translator',
            remappings=[
             ('/translator/cola2_nav', ["/", LaunchConfiguration("robot_name"), "/navigator/navigation"]),
             ('/translator/stonefish_dvl', ["/", LaunchConfiguration("robot_name"), "/navigator/dvl_sim"]),
             ('/translator/cola2_dvl', ["/", LaunchConfiguration("robot_name"), "/navigator/dvl"]),
             ('/translator/stonefish_thruster_setpoints', ["/", LaunchConfiguration("robot_name"), "/controller/thruster_setpoints_sim"]),
             ('/translator/cola2_thruster_setpoints', ["/", LaunchConfiguration("robot_name"), "/controller/thruster_setpoints"]),
             ('/translator/stonefish_navigation', ["/", LaunchConfiguration("robot_name"), "/navigator/navigation_sim"]),
    ])

    return LaunchDescription([
        # DeclareLaunchArgument('log_level', default_value='DEBUG', description='Set log level'),
        robot_name_arg,
        robot_package_arg,
        robot_description_arg,
        robot_xacro_arg,
        period_controller_arg,
        n_thrusters_arg,
        config_folder_arg,
        robot_state_publisher,
        
        GroupAction([
            PushRosNamespace(LaunchConfiguration('robot_name')),
            diagnostic_node,
            captain_node,
            controller_node,
            pilot_node,
            teleoperation_node,
            navigator_node,
            odom_tf_node,
            joy_node,
            logitech_node,
            messages_node,
            gui_server_node,
        ]),

        static_transform_publisher_node,
        ros2_stonefish_node,
        translator_node,
    ])