// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_pointcloud_utils/flatscan_to_laserscan_node.hpp"

#include "isaac_ros_common/qos.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include "rclcpp/logger.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace pointcloud_utils
{


FlatScantoLaserScanNode::FlatScantoLaserScanNode(rclcpp::NodeOptions options)
: Node("flatscan_to_laserscan", options.use_intra_process_comms(true)),
  // parameter
  angle_min_(declare_parameter<double>("angle_min", 0.0)),
  angle_max_(declare_parameter<double>("angle_max", 2 * M_PI)),
  angle_increment_(declare_parameter<double>("angle_increment", M_PI / 180)),
  time_increment_(declare_parameter<double>("time_increment", 0.0001)),
  max_range_fallback_(declare_parameter<double>("max_range_fallback", 200.0))
{
  // This function sets the QoS parameter for publishers and subscribers setup by this ROS node
  rclcpp::QoS input_qos_ = ::isaac_ros::common::AddQosParameter(
    *this, "DEFAULT", "input_qos");
  rclcpp::QoS output_qos_ = ::isaac_ros::common::AddQosParameter(
    *this, "DEFAULT", "output_qos");

  flatscan_sub_ = create_subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>(
    "flatscan", input_qos_, std::bind(
      &FlatScantoLaserScanNode::FlatScanCallback, this, std::placeholders::_1));
  laserscan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", output_qos_);
}

void FlatScantoLaserScanNode::FlatScanCallback(
  const isaac_ros_pointcloud_interfaces::msg::FlatScan::SharedPtr msg)
{
  auto laser_scan_message = sensor_msgs::msg::LaserScan();
  laser_scan_message.header = msg->header;
  laser_scan_message.angle_min = angle_min_;
  laser_scan_message.angle_max = angle_max_;
  laser_scan_message.angle_increment = angle_increment_;
  laser_scan_message.time_increment = time_increment_;
  laser_scan_message.scan_time = time_increment_ * msg->ranges.size();
  laser_scan_message.range_min = msg->range_min;
  if (msg->range_max != 0.0) {
    laser_scan_message.range_max = msg->range_max;
  } else {
    laser_scan_message.range_max = max_range_fallback_;
  }
  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (laser_scan_message.angle_max - laser_scan_message.angle_min) /
    laser_scan_message.angle_increment);
  // Intialize all values in the ranges array with laser_scan_message.range_max
  laser_scan_message.ranges.assign(ranges_size, laser_scan_message.range_max);
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float range = msg->ranges[i];
    float angle = msg->angles[i];
    // angle origin is shifted by the start angle which is laser_scan_message.angle_min
    angle = angle + laser_scan_message.angle_min;
    // Ensure that angles are within [0,2*M_PI)
    if (angle >= 2 * M_PI || angle <= -2 * M_PI) {
      angle = fmod(angle, (2 * M_PI));
    }
    if (angle < 0) {
      angle = angle + 2 * M_PI;
    }
    int index = angle / laser_scan_message.angle_increment;
    if (range < laser_scan_message.ranges[index]) {
      laser_scan_message.ranges[index] = range;
    }
  }
  laserscan_pub_->publish(laser_scan_message);
}


FlatScantoLaserScanNode::~FlatScantoLaserScanNode() = default;

}  // namespace pointcloud_utils
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::pointcloud_utils::FlatScantoLaserScanNode)
