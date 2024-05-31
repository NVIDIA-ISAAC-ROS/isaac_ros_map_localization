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
import pathlib
import time

from isaac_ros_pointcloud_interfaces.msg import FlatScan
from isaac_ros_test import IsaacROSBaseTest
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """
    Isaac ROS Point Cloud to Flatscan Converter.

    This test publishes "Point Clouds" from a rosbag
    and checks if a "flatscan" received at the output
    of the flatenner is of the expected size.
    """
    pointcloud_to_flatscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::PointCloudToFlatScanNode',
        name='pointcloud_to_flatscan',
        namespace=IsaacROSPointCloudToFlatScanPOLTest.generate_namespace()
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.path.dirname(__file__) +
             '/../data/rosbags/pointcloud', '-l', '-d', '4',
             '--remap',
             '/pointcloud:=' +
             IsaacROSPointCloudToFlatScanPOLTest.generate_namespace()
             + '/pointcloud'],
        output='screen'
    )

    pointcloud_to_flatscan_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='pointcloud_to_flatscan_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            pointcloud_to_flatscan_node,
        ],
        output='screen'
    )
    return IsaacROSPointCloudToFlatScanPOLTest.generate_test_description([
        rosbag_play,
        pointcloud_to_flatscan_container
    ])


class IsaacROSPointCloudToFlatScanPOLTest(IsaacROSBaseTest):
    """Test for Isaac ROS PointCloudToFlatScan Proof of Life."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_pointcloud_to_flatscan_pipeline(self) -> None:
        """Expect the pipeline to produce FlatScan data from a PointCloud."""
        self.generate_namespace_lookup(
            ['flatscan'])

        received_messages = {}
        flatscan_sub, = self.create_logging_subscribers(
            [('flatscan', FlatScan)], received_messages)
        try:
            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 20
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                # If we have received exactly one message on the output topic, break
                if 'flatscan' in received_messages:
                    done = True
                    break

            self.assertTrue(
                done, "Didn't receive output on flatscan topic!")

            flatscan_message = received_messages['flatscan']
            print(len(flatscan_message.ranges))
            self.assertTrue(len(flatscan_message.ranges) == 8892,
                            'Size of range arrray is not as expected!')

        finally:
            pass
            self.assertTrue(self.node.destroy_subscription(
                flatscan_sub))
