# Copyright (c) 2022，Horizon Robotics.
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

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_prefix('imu_sensor'),
        "lib/imu_sensor/config/bmi088.yaml")
    print("config_file_path is ", config_file_path)

    return LaunchDescription([
        # launch imu publish package
        Node(
            package='imu_sensor',
            executable='imu_sensor',
            output='screen',
            parameters=[
                {"config_file_path": str(config_file_path)},
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
