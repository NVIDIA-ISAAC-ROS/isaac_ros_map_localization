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

#ifndef ISAAC_ROS_OCCUPANCY_GRID_LOCALIZER__OCCUPANCY_GRID_LOCALIZER_NODE_HPP_
#define ISAAC_ROS_OCCUPANCY_GRID_LOCALIZER__OCCUPANCY_GRID_LOCALIZER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_pointcloud_interfaces/msg/flat_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace nvidia
{
namespace isaac_ros
{
namespace occupancy_grid_localizer
{

class OccupancyGridLocalizerNode : public nitros::NitrosNode
{
public:
  explicit OccupancyGridLocalizerNode(const rclcpp::NodeOptions &);

  ~OccupancyGridLocalizerNode();

  OccupancyGridLocalizerNode(const OccupancyGridLocalizerNode &) = delete;

  OccupancyGridLocalizerNode & operator=(const OccupancyGridLocalizerNode &) = delete;

  // callback used for overiding gxf parameters
  void postLoadGraphCallback() override;

  // callback used for overiding gxf parameters that is not supported by gxf functions
  void preLoadGraphCallback() override;

  void GridSearchLocalizationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>);

  void FlatScanCallback(
    const std::shared_ptr<isaac_ros_pointcloud_interfaces::msg::FlatScan> flat_scan);

  void LocalizationResultCallback(const gxf_context_t context, nitros::NitrosTypeBase & msg);

  // Callback linked to FlatScan subscriber
  // used to populate atlas with transform on atlas wrt to robot
  void FlatScanNitrosSubCallback(const gxf_context_t, nitros::NitrosTypeBase &);

  // Callback linked to Pose3dwitCov publisher
  // used to convert the result from Isaac map standard to ROS map_laoder standard
  void LocResultNitrosSubCallback(const gxf_context_t, nitros::NitrosTypeBase &);

private:
  rclcpp::Publisher<isaac_ros_pointcloud_interfaces::msg::FlatScan>::SharedPtr flat_scan_publisher_;

  rclcpp::Subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>::SharedPtr
    flat_scan_subscriber_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr grid_search_localize_service_server_;

  // read the image height without loading the image
  void GePngImageHeight(const std::string & file_path, unsigned int & height);

  const double cell_size_;
  const int occupancy_grid_map_threshold_;
  const std::string map_yaml_path_;
  const std::string map_png_path_;
  const std::vector<double> map_origin_;
  const int max_points_;
  const bool use_gxf_map_convention_;
  // The radius of the robot. This parameter is used to exclude poses which are too close to an
  // obstacle.
  const double robot_radius_;
  // The maximum beam error used when comparing range scans.
  const double max_beam_error_;
  // The max output error from our best sample, if output error larger than this threshold, we
  // conclude localization failed
  const double max_output_error_;
  // The minimal output error used to normalize and compute confidence, if output error from best
  // sample smaller or equal to this, the confidence is 1
  const double min_output_error_;
  // The GPU accelerated scan-and-match function can only handle a certain number of beams per
  // range scan. The allowed values are {32, 64, 128, 256, 512}. If the number of beams in the
  // range scan does not match this number a subset of beams will be taken.
  const int num_beams_gpu_;
  // This is the number of scans to collect into a batch for the GPU kernel. Choose a value which
  // matches your GPU well.
  const int batch_size_;
  // Distance between sample points in meters. The smaller this number, the more sample poses
  // will be considered. This leads to a higher accuracy and lower performance.
  const double sample_distance_;
  // Points range larger than this threshold will be marked as out of range and not used.
  const double out_of_range_threshold_;
  // Points range smaller than this threshold will be marked as invalid and not used.
  const double invalid_range_threshold_;
  // Minimal required scan fov to run the localizer.
  const double min_scan_fov_degrees_;
  // Whether or not pick the closest angle beam in angle bucket, if not pick the average within an
  // angular bucket
  const bool use_closest_beam_;

  unsigned int map_png_height_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // If true will trigger the localization when the next flatscan is received.
  std::atomic<bool> trigger_localization_on_next_flatscan_ = false;
};

}  // namespace occupancy_grid_localizer
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_OCCUPANCY_GRID_LOCALIZER__OCCUPANCY_GRID_LOCALIZER_NODE_HPP_
