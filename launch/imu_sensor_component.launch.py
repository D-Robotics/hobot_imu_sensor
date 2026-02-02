# Copyright (c) 2026，D-Robotics.
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
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes

def declare_configurable_parameters(parameters):
    args = []
    for p in parameters:
        kwargs = {
            "name": p["name"],
            "description": p.get("description", "")
        }
        if "default_value" in p and p["default_value"] is not None:
            kwargs["default_value"] = str(p["default_value"])
        args.append(DeclareLaunchArgument(**kwargs))
    return args


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    node_params = [
        {'name':'imu_pub_topic', 'default_value': "~/bmi08x_imu", 'description': 'imu_pub_topic'},
        {'name':'imu_iio_device', 'default_value': "/dev/iio:device1", 'description': 'imu_iio_device'},
        {'name':'imu_data_node', 'default_value': "/dev/input/event1", 'description': 'imu_data_node'},
        {'name':'imu_virtual_node', 'default_value': "/sys/devices/virtual/input/input1/", 'description': 'imu_config_node'},
        {'name':'imu_iic_bus', 'default_value': 5, 'description': 'imu_iic_bus'},
        {'name':'imu_acc_range', 'default_value': 12, 'description': 'imu_acc_range'},
        {'name':'imu_acc_bandwidth', 'default_value': 47, 'description': 'imu_acc_bandwidth'},
        {'name':'imu_gyro_range', 'default_value': 1000, 'description': 'imu_gyro_range'},
        {'name':'imu_gyro_bandwidth', 'default_value': 47, 'description': 'imu_gyro_bandwidth'},
        {'name':'imu_group_delay', 'default_value': 7, 'description': 'imu_group_delay'},
        {'name':'imu_gravity', 'default_value': 9.79494, 'description': 'imu_gravity'},
        {'name':'imu_frame_id', 'default_value': "imu_bmi088", 'description': 'imu_frame_id'},
        {'name':'imu_adjust_interrupt', 'default_value': False, 'description': 'imu_adjust_interrupt'},
        {'name':'imu_use_pool', 'default_value': False, 'description': 'imu_use_pool'},
        {'name':'imu_log_level', 'default_value': 'warn', 'description': 'imu_log_level'},
    ]

    launch = declare_configurable_parameters(node_params)
    launch.append(
        LoadComposableNodes(
            target_container=LaunchConfiguration("target_container_name"),
            composable_node_descriptions=[
                ComposableNode(
                    package="imu_sensor",
                    namespace='',
                    plugin="drobotics::ImuComponent",
                    name="drobotics_imu_component",
                    parameters=[set_configurable_parameters(node_params)],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ]
        )
    )

    return LaunchDescription(launch)
