# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# Import math Library
import math

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    flatscan_to_laserscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::FlatScantoLaserScanNode',
        name='flatscan_to_laserscan',
        parameters=[{'angle_min': 0.0,
                     'angle_max': 2*math.pi,
                     'angle_increment': math.pi/180,  # 0.0174533 rad => 1deg
                     'time_increment': 0.0001,
                     'max_range_fallback': 200.0}])

    flatscan_to_laserscan_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='flatscan_to_laserscan_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            flatscan_to_laserscan_node,
        ],
        output='screen'
    )

    return launch.LaunchDescription([flatscan_to_laserscan_container])
