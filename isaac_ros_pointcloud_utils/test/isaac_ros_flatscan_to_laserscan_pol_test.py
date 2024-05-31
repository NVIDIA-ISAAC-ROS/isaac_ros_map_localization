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

from isaac_ros_test import IsaacROSBaseTest
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy

from sensor_msgs.msg import LaserScan


@pytest.mark.rostest
def generate_test_description():
    """
    Isaac ROS Flatscan to Laserscan Converter.

    This test publishes "flatscan" from a rosbag
    and checks if a "laserscan" received at the output
    of the flatenner is of the expected size.
    """
    flatscan_to_laserscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::FlatScantoLaserScanNode',
        name='flatscan_to_laserscan',
        namespace=IsaacROSFlatScantoLaserScanPOLTest.generate_namespace()
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.path.dirname(__file__) +
             '/../data/rosbags/flatscan', '-l', '-d', '4',
             '--remap',
             '/flatscan:=' +
             IsaacROSFlatScantoLaserScanPOLTest.generate_namespace()
             + '/flatscan'],
        output='screen'
    )

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
    return IsaacROSFlatScantoLaserScanPOLTest.generate_test_description([
        rosbag_play,
        flatscan_to_laserscan_container
    ])


class IsaacROSFlatScantoLaserScanPOLTest(IsaacROSBaseTest):
    """Test for Isaac ROS FlatScanToLaserscan Proof of Life."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_flatscan_to_laserscan_pipeline(self) -> None:
        """Expect the pipeline to produce Laserscan data from a Flatscan."""
        self.generate_namespace_lookup(
            ['scan'])

        received_messages = {}
        laserscan_sub, = self.create_logging_subscribers(
            [('scan', LaserScan)], received_messages)
        try:
            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 20
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                # If we have received exactly one message on the output topic, break
                if 'scan' in received_messages:
                    done = True
                    break

            self.assertTrue(
                done, "Didn't receive output on laserscan topic!")

            laserscan_message = received_messages['scan']
            print(len(laserscan_message.ranges))
            self.assertTrue(len(laserscan_message.ranges) == 360,
                            'Size of range arrray is not as expected!')

        finally:
            pass
            self.assertTrue(self.node.destroy_subscription(
                laserscan_sub))
