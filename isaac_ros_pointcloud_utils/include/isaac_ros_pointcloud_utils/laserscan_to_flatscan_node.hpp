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

#ifndef ISAAC_ROS_POINTCLOUD_UTILS__LASERSCAN_TO_FLATSCAN_NODE_HPP_
#define ISAAC_ROS_POINTCLOUD_UTILS__LASERSCAN_TO_FLATSCAN_NODE_HPP_

#include "isaac_ros_pointcloud_interfaces/msg/flat_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace pointcloud_utils
{

class LaserScantoFlatScanNode : public rclcpp::Node
{
public:
  explicit LaserScantoFlatScanNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~LaserScantoFlatScanNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
  rclcpp::Publisher<isaac_ros_pointcloud_interfaces::msg::FlatScan>::SharedPtr flatscan_pub_;

  void LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

}  // namespace pointcloud_utils
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_POINTCLOUD_UTILS__LASERSCAN_TO_FLATSCAN_NODE_HPP_
