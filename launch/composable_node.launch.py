# -----------------------------------------------------------------------------
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Create composable node in container."""
    container = ComposableNodeContainer(
        name='arduino_imu_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='arduino_imu',
                plugin='arduino_imu::ArduinoImu',
                name='arduino_imu',
                parameters=[
                    {'device': '/dev/ttyACM0',
                     'baud': 500000,
                     # 'baud': 115200,
                     'queue_size': 10}],
                remappings=[],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen')
    return [container]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription([
        OpaqueFunction(function=launch_setup)
        ])
