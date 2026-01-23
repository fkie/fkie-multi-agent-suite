# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile


def generate_launch_description():
    capability_group = LaunchConfiguration('capability_group')

    remappings = []

    declare_capability_group_cmd = DeclareLaunchArgument(
        'capability_group', default_value='2 py', description='Capability group of all nodes'
    )

    load_nodes = GroupAction(
        actions=[
            Node(
                package='examples_rclcpp_minimal_publisher',
                executable='publisher_not_composable',
                name='talker',
                output='screen',
                parameters=[{'capability_group': capability_group}],
                remappings=remappings + [('topic', 'topic_test')],
            ),
            Node(
                package='examples_rclcpp_minimal_subscriber',
                executable='subscriber_not_composable',
                name='listener',
                output='screen',
                parameters=[{'capability_group': capability_group}],
                remappings=remappings + [('topic', 'topic_test')],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_capability_group_cmd)
    # Add the actions to launch all of nodes
    ld.add_action(load_nodes)

    return ld
