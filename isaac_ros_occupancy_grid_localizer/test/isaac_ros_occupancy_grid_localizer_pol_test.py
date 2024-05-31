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

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovarianceStamped
from isaac_ros_test import IsaacROSBaseTest
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy
from std_srvs.srv import Empty

TEST_POS_TOLERANCE = 0.15
TEST_QUAT_TOLERANCE = 0.013
TEST_COV_TOLERANCE = 0.001


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    map_yaml_path = os.path.join(
        get_package_share_directory('isaac_ros_occupancy_grid_localizer'), 'maps', 'map.yaml')

    occupancy_grid_localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        namespace=IsaacROSOccupancyGridLocalizerPOLTest.generate_namespace(),
        parameters=[map_yaml_path, {
            'loc_result_frame': 'map',
            'map_yaml_path': map_yaml_path,
        }]
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.path.dirname(__file__) +
             '/../data/rosbags/flatscan', '-l',
             '--remap',
             '/flatscan:=' +
             IsaacROSOccupancyGridLocalizerPOLTest.generate_namespace()
             + '/flatscan'],
        output='screen'
    )

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
    return IsaacROSOccupancyGridLocalizerPOLTest.generate_test_description([
        occupancy_grid_localizer_container,
        rosbag_play
    ])


class IsaacROSOccupancyGridLocalizerPOLTest(IsaacROSBaseTest):
    """Test for Isaac ROS OccupancyGridLocalizer Proof of Life."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_occupancy_grid_localizer_pipeline(self) -> None:
        """Expect the pipeline to produce Intialpose data from scan and map."""
        self.generate_namespace_lookup(
            ['localization_result'])

        received_messages = {}
        localization_result_sub, = self.create_logging_subscribers(
            [('localization_result', PoseWithCovarianceStamped)], received_messages,
            accept_multiple_messages=True)
        # Create service client
        self.cli = self.node.create_client(
            Empty, '/isaac_ros_test/trigger_grid_search_localization')
        self.req = Empty.Request()
        # Check if the a service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        try:
            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 20
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                self.future = self.cli.call_async(self.req)
                rclpy.spin_until_future_complete(self.node, self.future)
                # If we have received exactly one message on the output topic, break
                if len(received_messages['localization_result']) > 0:
                    done = True
                    break

            self.assertTrue(
                done, "Didn't receive output on localization_result topic!")

            self.node._logger.info('At least one message was received.')

            loc_result_actual = received_messages['localization_result'][0]
            self.assertAlmostEqual(loc_result_actual.pose.pose.position.x,
                                   33.5, None,
                                   'received position X coordinate is not within tolerance',
                                   TEST_POS_TOLERANCE)
            self.assertAlmostEqual(loc_result_actual.pose.pose.position.y,
                                   7.75, None,
                                   'received position Y coordinate is not within tolerance',
                                   TEST_POS_TOLERANCE)
            self.assertAlmostEqual(loc_result_actual.pose.pose.position.z,
                                   0.0, None,
                                   'received position Z coordinate is not within tolerance',
                                   TEST_POS_TOLERANCE)
            self.assertAlmostEqual(loc_result_actual.pose.pose.orientation.x,
                                   0.0, None,
                                   'received quaternion X is not within tolerance',
                                   TEST_QUAT_TOLERANCE)
            self.assertAlmostEqual(loc_result_actual.pose.pose.orientation.y,
                                   0.0, None,
                                   'received quaternion Y is not within tolerance',
                                   TEST_QUAT_TOLERANCE)
            self.assertAlmostEqual(loc_result_actual.pose.pose.orientation.z,
                                   -0.56573, None,
                                   'received quaternion Z is not within tolerance',
                                   TEST_QUAT_TOLERANCE)
            self.assertAlmostEqual(loc_result_actual.pose.pose.orientation.w,
                                   0.824589, None,
                                   'received quaternion W is not within tolerance',
                                   TEST_QUAT_TOLERANCE)
            for i in range(36):
                self.assertAlmostEqual(loc_result_actual.pose.covariance[i],
                                       0.0, None,
                                       'received covariance is not within tolerance',
                                       TEST_COV_TOLERANCE)

            self.node._logger.info('Received message was verified successfully.')

        finally:
            pass
            self.assertTrue(self.node.destroy_subscription(
                localization_result_sub))
