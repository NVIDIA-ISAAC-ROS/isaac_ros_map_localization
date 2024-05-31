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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    lifecycle_nodes = ['map_server']
    map_yaml_path_dir = os.path.join(
        get_package_share_directory('isaac_ros_occupancy_grid_localizer'), 'maps', 'map.yaml')

    occupancy_grid_localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        parameters=[map_yaml_path_dir, {
            'loc_result_frame': 'map',
            'map_yaml_path': map_yaml_path_dir,
        }])

    occupancy_grid_localizer_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='occupancy_grid_localizer_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            occupancy_grid_localizer_node,
        ],
        output='screen'
    )

    # Seting transform between lidar_frame and base_link
    # since Isaac Sim does not set this transform
    baselink_lidar_publisher = Node(
        package='tf2_ros', executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link',
                   'lidar_frame'])

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[{'yaml_filename': map_yaml_path_dir}]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription([occupancy_grid_localizer_container,
                            baselink_lidar_publisher])

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_nodes)

    return ld
