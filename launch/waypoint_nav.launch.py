# Copyright 2026 RHIT Thornbots
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

"""Launch waypoint_nav with config/route.yaml; see README.md."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('thornbots_nav')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'route.yaml'),
            description='Route parameters YAML.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock; NAV_GOAL stamps follow the node clock.'),
        Node(
            package='thornbots_nav',
            executable='waypoint_nav',
            name='waypoint_nav',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
