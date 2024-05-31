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

#include "isaac_ros_pointcloud_utils/laserscan_to_flatscan_node.hpp"

#include "isaac_ros_common/qos.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/logger.hpp"

namespace
{
constexpr int LASERSCAN_N_DIFF_TOL_ = 1;
}

namespace nvidia
{
namespace isaac_ros
{
namespace pointcloud_utils
{


LaserScantoFlatScanNode::LaserScantoFlatScanNode(rclcpp::NodeOptions options)
: Node("laserscan_to_flatscan", options.use_intra_process_comms(true))
{
  // This function sets the QoS parameter for publishers and subscribers setup by this ROS node
  rclcpp::QoS input_qos_ = ::isaac_ros::common::AddQosParameter(
    *this, "DEFAULT", "input_qos");
  rclcpp::QoS output_qos_ = ::isaac_ros::common::AddQosParameter(
    *this, "DEFAULT", "output_qos");

  laserscan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", input_qos_, std::bind(
      &LaserScantoFlatScanNode::LaserScanCallback, this, std::placeholders::_1));
  flatscan_pub_ =
    create_publisher<isaac_ros_pointcloud_interfaces::msg::FlatScan>("flatscan", output_qos_);
}

void LaserScantoFlatScanNode::LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto flatscan_message = isaac_ros_pointcloud_interfaces::msg::FlatScan();
  flatscan_message.header = msg->header;
  flatscan_message.range_min = msg->range_min;
  flatscan_message.range_max = msg->range_max;

  bool copy_intensities = (msg->ranges.size() == msg->intensities.size()) ? true : false;
  float angle = msg->angle_min;
  int expected_ranges_size = std::ceil(
    (msg->angle_max - msg->angle_min) /
    msg->angle_increment);
  if (std::abs(expected_ranges_size - static_cast<int>(msg->ranges.size())) >
    LASERSCAN_N_DIFF_TOL_)
  {
    RCLCPP_WARN(
      this->get_logger(), "The angle specs and range array size does not match: %u, %lu",
      expected_ranges_size,
      msg->ranges.size());
  }
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    flatscan_message.ranges.push_back(msg->ranges[i]);
    flatscan_message.angles.push_back(angle);
    if (copy_intensities) {
      flatscan_message.intensities.push_back(msg->intensities[i]);
    }
    angle += msg->angle_increment;
  }
  flatscan_pub_->publish(flatscan_message);
}


LaserScantoFlatScanNode::~LaserScantoFlatScanNode() = default;

}  // namespace pointcloud_utils
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode)
