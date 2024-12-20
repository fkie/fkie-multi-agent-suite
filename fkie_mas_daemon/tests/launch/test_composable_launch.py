import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('fkie_mas_daemon')
    launch_dir = os.path.join(bringup_dir, 'test', 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    yaml_file = LaunchConfiguration('yaml_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    example_envvar = SetEnvironmentVariable(
        'EXAMPLE_ENV_VAR', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_yaml_file = DeclareLaunchArgument(
        'yaml_file', default_value='', description='Full path to yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level', choices=['info', 'debug', 'error']
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                condition=IfCondition(use_composition),
                name='my_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[{'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, 'slam_launch.py')
            #     ),
            #     condition=IfCondition(PythonExpression([slam, ' and ', use_localization])),
            #     launch_arguments={
            #         'namespace': namespace,
            #         'use_sim_time': use_sim_time,
            #         'autostart': autostart,
            #         'use_respawn': use_respawn,
            #         'params_file': params_file,
            #     }.items(),
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, 'localization_launch.py')
            #     ),
            #     condition=IfCondition(PythonExpression(['not ', slam, ' and ', use_localization])),
            #     launch_arguments={
            #         'namespace': namespace,
            #         'map': map_yaml_file,
            #         'use_sim_time': use_sim_time,
            #         'autostart': autostart,
            #         'params_file': params_file,
            #         'use_composition': use_composition,
            #         'use_respawn': use_respawn,
            #         'container_name': 'nav2_container',
            #     }.items(),
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'test_included_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': yaml_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'my_container',
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(example_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_yaml_file)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
