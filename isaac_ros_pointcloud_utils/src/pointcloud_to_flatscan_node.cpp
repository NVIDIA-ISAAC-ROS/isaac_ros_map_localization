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

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "isaac_ros_common/qos.hpp"

#include "isaac_ros_pointcloud_utils/pointcloud_to_flatscan_node.hpp"
#include "isaac_ros_nitros_flat_scan_type/nitros_flat_scan.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

namespace
{
// number of components in PointCloud gxf msg
constexpr int kPointCloudGxfMsgSize = 4;
constexpr int kMsgBufferSize = 5;
}

namespace nvidia
{
namespace isaac_ros
{
namespace pointcloud_utils
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

constexpr char INPUT_COMPONENT_KEY_POINTCLOUD[] = "pointcloud_to_flatscan/rx_point_cloud";
constexpr char INPUT_DEFAULT_TENSOR_FORMAT_POINTCLOUD[] = "nitros_point_cloud";
constexpr char INPUT_TOPIC_NAME_POINTCLOUD[] = "pointcloud";

constexpr char OUTPUT_COMPONENT_KEY_FLATSCAN[] = "sink/sink";
constexpr char OUTPUT_DEFAULT_TENSOR_FORMAT_FLATSCAN[] = "nitros_flat_scan";
constexpr char OUTPUT_TOPIC_NAME_FLATSCAN[] = "flatscan";

constexpr char APP_YAML_FILENAME[] = "config/pointcloud_to_flatscan_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_pointcloud_utils";

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"gxf_isaac_point_cloud", "gxf/lib/libgxf_isaac_point_cloud.so"}
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_pointcloud_utils"
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {INPUT_COMPONENT_KEY_POINTCLOUD,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = INPUT_DEFAULT_TENSOR_FORMAT_POINTCLOUD,
      .topic_name = INPUT_TOPIC_NAME_POINTCLOUD,
    }
  },
  {OUTPUT_COMPONENT_KEY_FLATSCAN,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_FLATSCAN,
      .topic_name = OUTPUT_TOPIC_NAME_FLATSCAN,
      .frame_id_source_key = INPUT_COMPONENT_KEY_POINTCLOUD,
    }
  }
};
#pragma GCC diagnostic pop

PointCloudToFlatScanNode::PointCloudToFlatScanNode(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME),
  min_x_(declare_parameter<double>("min_x", -1.0)),
  max_x_(declare_parameter<double>("max_x", 1.0)),
  min_y_(declare_parameter<double>("min_y", -1.0)),
  max_y_(declare_parameter<double>("max_y", 1.0)),
  min_z_(declare_parameter<double>("min_z", -0.1)),
  max_z_(declare_parameter<double>("max_z", 0.1)),
  max_points_(declare_parameter<double>("max_points", 150000)),
  threshold_x_axis_(declare_parameter<bool>("threshold_x_axis", false)),
  threshold_y_axis_(declare_parameter<bool>("threshold_y_axis", false))
{
  RCLCPP_DEBUG(get_logger(), "[PointCloudToFlatScanNode] Constructor");

  // This function sets the QoS parameter for publishers and subscribers setup by this NITROS node
  rclcpp::QoS input_qos_ = ::isaac_ros::common::AddQosParameter(
    *this, "DEFAULT", "input_qos");
  rclcpp::QoS output_qos_ = ::isaac_ros::common::AddQosParameter(
    *this, "DEFAULT", "output_qos");
  for (auto & config : config_map_) {
    if (config.second.topic_name == INPUT_TOPIC_NAME_POINTCLOUD) {
      config.second.qos = input_qos_;
    } else {
      config.second.qos = output_qos_;
    }
  }

  registerSupportedType<nvidia::isaac_ros::nitros::NitrosFlatScan>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosPointCloud>();

  startNitrosNode();
}

void PointCloudToFlatScanNode::postLoadGraphCallback()
{
  RCLCPP_DEBUG(get_logger(), "[PointCloudToFlatScanNode] postLoadGraphCallback().");
  getNitrosContext().setParameterFloat64(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan", "min_x",
    min_x_);
  getNitrosContext().setParameterFloat64(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan", "max_x",
    max_x_);
  getNitrosContext().setParameterFloat64(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan", "min_y",
    min_y_);
  getNitrosContext().setParameterFloat64(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan", "max_y",
    max_y_);
  getNitrosContext().setParameterFloat64(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan", "min_z",
    min_z_);
  getNitrosContext().setParameterFloat64(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan", "max_z",
    max_z_);
  getNitrosContext().setParameterInt32(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan", "max_points",
    max_points_);
  getNitrosContext().setParameterBool(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan",
    "threshold_x_axis",
    threshold_x_axis_);
  getNitrosContext().setParameterBool(
    "pointcloud_to_flatscan", "nvidia::isaac_ros::point_cloud::PointCloudToFlatscan",
    "threshold_y_axis",
    threshold_y_axis_);
  // decide how much memory to allocate based on how much memory used in
  // PointCloudToFlatscan source code
  getNitrosContext().setParameterUInt64(
    "pointcloud_to_flatscan", "nvidia::gxf::BlockMemoryPool",
    "block_size",
    kPointCloudGxfMsgSize * kMsgBufferSize * sizeof(float) * max_points_ + sizeof(uint));
}

PointCloudToFlatScanNode::~PointCloudToFlatScanNode() {}

}  // namespace pointcloud_utils
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::pointcloud_utils::PointCloudToFlatScanNode)
