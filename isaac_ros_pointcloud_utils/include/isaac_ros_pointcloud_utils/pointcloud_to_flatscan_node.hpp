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

#ifndef ISAAC_ROS_POINTCLOUD_UTILS__POINTCLOUD_TO_FLATSCAN_NODE_HPP_
#define ISAAC_ROS_POINTCLOUD_UTILS__POINTCLOUD_TO_FLATSCAN_NODE_HPP_

#include <string>
#include <chrono>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace pointcloud_utils
{

class PointCloudToFlatScanNode : public nitros::NitrosNode
{
public:
  explicit PointCloudToFlatScanNode(const rclcpp::NodeOptions &);

  // The callback to be implemented by users for any required initialization
  void postLoadGraphCallback() override;

  ~PointCloudToFlatScanNode();

  PointCloudToFlatScanNode(const PointCloudToFlatScanNode &) = delete;

  PointCloudToFlatScanNode & operator=(const PointCloudToFlatScanNode &) = delete;

private:
  const double min_x_;
  const double max_x_;
  const double min_y_;
  const double max_y_;
  const double min_z_;
  const double max_z_;
  const int max_points_;
  const bool threshold_x_axis_;
  const bool threshold_y_axis_;
};

}  // namespace pointcloud_utils
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_POINTCLOUD_UTILS__POINTCLOUD_TO_FLATSCAN_NODE_HPP_
