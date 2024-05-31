# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value=os.path.join(
            get_package_share_directory(
                'isaac_ros_occupancy_grid_localizer'), 'maps', 'isaac_sim.yaml'),
        description='Full path to map file to load')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            get_package_share_directory(
                'isaac_ros_occupancy_grid_localizer'), 'params', 'carter_nav2.yaml'),
        description='Full path to param file to load')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Omniverse Isaac Sim) clock if true')
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='True',
        description='Whether to start RVIZ')
    run_nav2_arg = DeclareLaunchArgument(
        'run_nav2', default_value='True',
        description='Whether to run nav2')

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(get_package_share_directory(
        'isaac_ros_occupancy_grid_localizer'), 'rviz',
        'isaac_sim_nav2.rviz')

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'autostart': 'True',
                          'rviz_config': rviz_config_dir
                          }.items(),
        condition=IfCondition(LaunchConfiguration('run_rviz')))

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_nav2'))
    )

    occupancy_grid_localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        parameters=[LaunchConfiguration('map_file'), {
            'loc_result_frame': 'map',
            'map_yaml_path': LaunchConfiguration('map_file'),
        }],
        remappings=[('localization_result', '/initialpose')])

    laserscan_to_flatscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode',
        name='laserscan_to_flatscan')

    occupancy_grid_localizer_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='occupancy_grid_localizer_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            occupancy_grid_localizer_node,
            laserscan_to_flatscan_node
        ],
        output='screen'
    )

    baselink_basefootprint_publisher = Node(
        package='tf2_ros', executable='static_transform_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        arguments=['0.0', '0.0', '0', '0.0', '0.0', '0.0', 'base_link', 'base_footprint'],
        condition=IfCondition(LaunchConfiguration('run_nav2')))

    return LaunchDescription([
        map_file_arg,
        params_file_arg,
        use_sim_time_arg,
        run_rviz_arg,
        run_nav2_arg,
        rviz_launch,
        nav2_launch,
        baselink_basefootprint_publisher,
        occupancy_grid_localizer_container
    ])
