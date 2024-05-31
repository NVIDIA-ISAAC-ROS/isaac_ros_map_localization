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

#include <filesystem>
namespace fs = std::filesystem;

#include "isaac_ros_common/qos.hpp"

#include "isaac_ros_occupancy_grid_localizer/occupancy_grid_localizer_node.hpp"
#include "isaac_ros_nitros_pose_cov_stamped_type/nitros_pose_cov_stamped.hpp"
#include "isaac_ros_nitros_flat_scan_type/nitros_flat_scan.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gems/core/math/pose3.hpp"
#include "gems/core/math/types.hpp"
#include "extensions/atlas/pose_tree_frame.hpp"
#include "gems/common/pose_frame_uid.hpp"
#include "gems/pose_tree/pose_tree.hpp"
#pragma GCC diagnostic pop

namespace
{
constexpr char kPoseTreeEntityName[] = "global_pose_tree";
constexpr char kPoseTreeComponentName[] = "pose_tree";
constexpr char kPoseTreeComponentTypeName[] = "nvidia::isaac::PoseTree";
constexpr char kRobotFrameComponentName[] = "robot_frame";
constexpr char kRobotFrameComponentTypeName[] = "nvidia::isaac::PoseTreeFrame";
// Name for the PoseFrameUid in FlatScan message
constexpr char kNamePoseFrameUid[] = "pose_frame_uid";
constexpr char kBaseLinkFrameName[] = "base_link";
// Name for the Pose3d in Pose3dCov message
constexpr char kPoseName[] = "pose";
// Image pixel max value
constexpr int kPixelMax = 255;
// number of components in FlatScan gxf msg
constexpr int kFlatScanGxfMsgSize = 5;
constexpr int kMsgBufferSize = 10;
}  // namespace

namespace nvidia
{
namespace isaac_ros
{
namespace occupancy_grid_localizer
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

constexpr char INPUT_COMPONENT_KEY_FLATSCAN[] = "flatscan_beams_tensor_copier/rx_flatscan";
constexpr char INPUT_DEFAULT_TENSOR_FORMAT_FLATSCAN[] = "nitros_flat_scan";
constexpr char INPUT_TOPIC_NAME_FLATSCAN[] = "flatscan_localization";

constexpr char OUTPUT_COMPONENT_KEY_LOC_RESULT[] = "sink/sink";
constexpr char OUTPUT_DEFAULT_TENSOR_FORMAT_LOC_RESULT[] = "nitros_pose_cov_stamped";
constexpr char OUTPUT_TOPIC_NAME_LOC_RESULT[] = "localization_result";
constexpr char OUTPUT_FRAME_ID_MAP_KEY[] = "map";

constexpr char APP_YAML_FILENAME[] = "config/occupancy_grid_localizer_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_occupancy_grid_localizer";

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  {"gxf_isaac_cuda", "gxf/lib/libgxf_isaac_cuda.so"},
  {"gxf_isaac_ros_cuda", "gxf/lib/libgxf_isaac_ros_cuda.so"},
  {"gxf_isaac_utils", "gxf/lib/libgxf_isaac_utils.so"},
  {"gxf_isaac_localization", "gxf/lib/libgxf_isaac_localization.so"},
  {"gxf_isaac_flatscan_localization", "gxf/lib/libgxf_isaac_flatscan_localization.so"}
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_occupancy_grid_localizer",
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule.yaml"
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {INPUT_COMPONENT_KEY_FLATSCAN,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = INPUT_DEFAULT_TENSOR_FORMAT_FLATSCAN,
      .topic_name = INPUT_TOPIC_NAME_FLATSCAN,
    }
  },
  {OUTPUT_COMPONENT_KEY_LOC_RESULT,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_LOC_RESULT,
      .topic_name = OUTPUT_TOPIC_NAME_LOC_RESULT,
      .frame_id_source_key = OUTPUT_FRAME_ID_MAP_KEY,
    }
  }
};
#pragma GCC diagnostic pop

OccupancyGridLocalizerNode::OccupancyGridLocalizerNode(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME),
  cell_size_(declare_parameter<double>("resolution", 0.05)),
  occupancy_grid_map_threshold_(kPixelMax * declare_parameter<double>("occupied_thresh", 0.65)),
  map_yaml_path_(declare_parameter<std::string>("map_yaml_path", "")),
  map_png_path_(map_yaml_path_.substr(0, map_yaml_path_.find_last_of(
      "/\\")) + "/" + declare_parameter<std::string>("image", "")),
  map_origin_(declare_parameter<std::vector<double>>("origin", {0.0, 0.0, 0.0})),
  max_points_(declare_parameter<int>("max_points", 20000)),
  use_gxf_map_convention_(declare_parameter<bool>("use_gxf_map_convention", false)),
  robot_radius_(declare_parameter<double>("robot_radius", 0.25)),
  max_beam_error_(declare_parameter<double>("max_beam_error", 0.5)),
  max_output_error_(declare_parameter<double>("max_output_error", 0.35)),
  min_output_error_(declare_parameter<double>("min_output_error", 0.22)),
  num_beams_gpu_(declare_parameter<int>("num_beams_gpu", 512)),
  batch_size_(declare_parameter<int>("batch_size", 512)),
  sample_distance_(declare_parameter<double>("sample_distance", 0.1)),
  out_of_range_threshold_(declare_parameter<double>("out_of_range_threshold", 100.0)),
  invalid_range_threshold_(declare_parameter<double>("invalid_range_threshold", 0.0)),
  min_scan_fov_degrees_(declare_parameter<double>("min_scan_fov_degrees", 270.0)),
  use_closest_beam_(declare_parameter<bool>("use_closest_beam", true))
{
  RCLCPP_DEBUG(get_logger(), "[OccupancyGridLocalizerNode] Constructor");

  // This function sets the QoS parameter for publishers and subscribers setup by this NITROS node
  rclcpp::QoS input_qos_ = ::isaac_ros::common::AddQosParameter(
    *this, "DEFAULT",
    "input_qos");
  rclcpp::QoS output_qos_ = ::isaac_ros::common::AddQosParameter(
    *this, "DEFAULT",
    "output_qos");
  for (auto & config : config_map_) {
    if (config.second.topic_name == INPUT_TOPIC_NAME_FLATSCAN) {
      config.second.qos = input_qos_;
    } else {
      config.second.qos = output_qos_;
    }
  }

  // Initialize tf buffer
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Input Flatscan msg callback to populate atlas with transform wrt to robot frame
  config_map_[INPUT_COMPONENT_KEY_FLATSCAN].callback =
    std::bind(
    &OccupancyGridLocalizerNode::FlatScanNitrosSubCallback, this,
    std::placeholders::_1, std::placeholders::_2);

  // Pose3DCov callback to convert map frame conventions from Isaac to ROS
  if (!use_gxf_map_convention_) {
    config_map_[OUTPUT_COMPONENT_KEY_LOC_RESULT].callback =
      std::bind(
      &OccupancyGridLocalizerNode::LocResultNitrosSubCallback, this,
      std::placeholders::_1, std::placeholders::_2);
  }

  // Exit with error if map is not specified
  if (!fs::exists(map_png_path_)) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not find map at %s. Exiting.",
      map_png_path_.c_str());
    throw std::runtime_error("Parameter parsing failure.");
  }

  GePngImageHeight(map_png_path_, map_png_height_);

  registerSupportedType<nvidia::isaac_ros::nitros::NitrosPoseCovStamped>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosFlatScan>();

  setFrameIdSource(
    OUTPUT_FRAME_ID_MAP_KEY,
    declare_parameter<std::string>("loc_result_frame", "map"));
  grid_search_localize_service_server_ =
    this->create_service<std_srvs::srv::Empty>(
    "trigger_grid_search_localization",
    std::bind(
      &OccupancyGridLocalizerNode::GridSearchLocalizationCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  flat_scan_subscriber_ =
    this->create_subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>(
    "flatscan", input_qos_,
    std::bind(
      &OccupancyGridLocalizerNode::FlatScanCallback, this,
      std::placeholders::_1));
  flat_scan_publisher_ =
    this->create_publisher<isaac_ros_pointcloud_interfaces::msg::FlatScan>(
    INPUT_TOPIC_NAME_FLATSCAN, input_qos_);
  startNitrosNode();
}

void OccupancyGridLocalizerNode::GePngImageHeight(
  const std::string & file_path,
  unsigned int & height)
{
  // The width is a 4-byte integer starting at offset 16 in the file.
  // See below link for more details
  // https://stackoverflow.com/questions/5354459/
  // c-how-to-get-the-image-size-of-a-png-file-in-directory/5354657#5354657
  unsigned char buf[8];

  std::ifstream in(file_path);
  in.seekg(16);
  in.read(reinterpret_cast<char *>(&buf), 8);

  height = (buf[4] << 24) + (buf[5] << 16) + (buf[6] << 8) + (buf[7] << 0);
}

void OccupancyGridLocalizerNode::GridSearchLocalizationCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  trigger_localization_on_next_flatscan_ = true;
  RCLCPP_INFO(
    get_logger(),
    "[OccupancyGridLocalizerNode] Will trigger localization when next flatscan is received.");
}

void OccupancyGridLocalizerNode::FlatScanCallback(
  const std::shared_ptr<isaac_ros_pointcloud_interfaces::msg::FlatScan> flat_scan)
{
  if (trigger_localization_on_next_flatscan_) {
    flat_scan_publisher_->publish(*flat_scan);
    RCLCPP_INFO(get_logger(), "[OccupancyGridLocalizerNode] Triggering localization now.");
    trigger_localization_on_next_flatscan_ = false;
  }
}

void OccupancyGridLocalizerNode::FlatScanNitrosSubCallback(
  const gxf_context_t context,
  nitros::NitrosTypeBase & msg)
{
  auto msg_entity = nvidia::gxf::Entity::Shared(context, msg.handle);
  // Get sensor pose id. It should have been populated by the type adapter
  // or source gxf codelet
  auto maybe_pose_frame_uid = msg_entity->get<nvidia::isaac::PoseFrameUid>(kNamePoseFrameUid);
  if (!maybe_pose_frame_uid) {
    std::stringstream error_msg;
    error_msg <<
      "[OccupancyGridLocalizerNode] Could not get PoseFrameUid in gxf message: " <<
      GxfResultStr(maybe_pose_frame_uid.error());
    RCLCPP_ERROR(
      get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_frame_uid = maybe_pose_frame_uid.value();

  // Get pointer to posetree component
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kPoseTreeEntityName, kPoseTreeComponentName, kPoseTreeComponentTypeName, cid);
  auto maybe_pose_tree_handle =
    nvidia::gxf::Handle<nvidia::isaac::PoseTree>::Create(context, cid);
  if (!maybe_pose_tree_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[OccupancyGridLocalizerNode] Failed to get pose tree's handle: " <<
      GxfResultStr(maybe_pose_tree_handle.error());
    RCLCPP_ERROR(
      get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();

  // Get frame name from atlas. It should have been populated by the type adapter
  // or source gxf codelet
  auto maybe_frame_name = pose_tree_handle->getFrameName(pose_frame_uid->uid);
  if (!maybe_frame_name) {
    std::stringstream error_msg;
    error_msg <<
      "[OccupancyGridLocalizerNode] Cannot not find Pose Tree Frame";
    RCLCPP_ERROR(
      get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto frame_name = maybe_frame_name.value();

  // Look up the last transformation between base_link and FlatScan frame_id
  // Wil
  geometry_msgs::msg::TransformStamped baselink_to_lidar_transform;
  try {
    baselink_to_lidar_transform = tf_buffer_->lookupTransform(
      kBaseLinkFrameName, frame_name,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s. Using Identity transform",
      kBaseLinkFrameName, frame_name, ex.what());
  }

  // Get pointer to posetree component
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kPoseTreeEntityName, kRobotFrameComponentName, kRobotFrameComponentTypeName, cid);
  auto maybe_robot_frame_handle =
    nvidia::gxf::Handle<nvidia::isaac::PoseTreeFrame>::Create(context, cid);
  if (!maybe_robot_frame_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[OccupancyGridLocalizerNode] Failed to get pose tree's handle: " <<
      GxfResultStr(maybe_robot_frame_handle.error());
    RCLCPP_ERROR(
      get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto robot_frame_handle = maybe_robot_frame_handle.value();
  auto robot_frame_uid = robot_frame_handle->frame_uid();

  // Populate atlas with transformation between base_link
  // and FlatScan frame_id
  pose_tree_handle->set(
    robot_frame_uid, pose_frame_uid->uid, 0.0,
    ::nvidia::isaac::Pose3d{
          ::nvidia::isaac::SO3d::FromQuaternion(
            ::nvidia::isaac::Quaterniond{
            baselink_to_lidar_transform.transform.rotation.w,
            baselink_to_lidar_transform.transform.rotation.x,
            baselink_to_lidar_transform.transform.rotation.y,
            baselink_to_lidar_transform.transform.rotation.z}),
          ::nvidia::isaac::Vector3d(
            baselink_to_lidar_transform.transform.translation.x,
            baselink_to_lidar_transform.transform.translation.y,
            baselink_to_lidar_transform.transform.translation.z)}
  );
}

// The below callback is used to convert the result from the Isaac
// to ROS map origin conventions.
void OccupancyGridLocalizerNode::LocResultNitrosSubCallback(
  const gxf_context_t context,
  nitros::NitrosTypeBase & msg)
{
  auto msg_entity = nvidia::gxf::Entity::Shared(context, msg.handle);
  // Get pose
  auto maybe_pose = msg_entity->get<::nvidia::isaac::Pose3d>(kPoseName);
  if (!maybe_pose) {
    std::stringstream error_msg;
    error_msg <<
      "[OccupancyGridLocalizerNode] Could not get Pose3d in gxf message: " <<
      GxfResultStr(maybe_pose.error());
    RCLCPP_ERROR(
      get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_handle = maybe_pose.value();

  // ROS map_server convention is that the origin is the bottom left corner,
  // with the X-axis in pointing right and the Y-axis pointing to the up.
  // Isaac convention is that the origin is the top left corner,
  // with the X-axis in pointing down and the Y-axis pointing to the right.
  // Hence the transformation matrix from the Isaac map frame to the
  // ROS map frame is as shown below
  auto ros_pose_isaac_transform = ::nvidia::isaac::Pose3d{
    ::nvidia::isaac::SO3d::FromQuaternion(
      ::nvidia::isaac::Quaterniond{0.7071068, 0.0, 0.0, -0.7071068}),
    ::nvidia::isaac::Vector3d(0.0, map_png_height_ * cell_size_, 0.0)
  };

  auto ros_map_origin_transform =
    ::nvidia::isaac::Pose3d::FromPose2XY(
    {::nvidia::isaac::SO2d::FromAngle(
        map_origin_[2]), {map_origin_[0], map_origin_[1]}});

  // The ros_pose_isaac_transform transformation matrix is
  // applied so that the published output conforms to ROS map conventions
  *pose_handle = ros_map_origin_transform * ros_pose_isaac_transform * (*pose_handle);
}

void OccupancyGridLocalizerNode::preLoadGraphCallback()
{
  RCLCPP_INFO(get_logger(), "[OccupancyGridLocalizerNode] preLoadGraphCallback().");
  // Setting gxf parameters not supported by gxf functions
  NitrosNode::preLoadGraphSetParameter(
    "binary_occupancy_map_loader", "nvidia::isaac::OccupancyGridMap", "sample_distance",
    std::to_string(sample_distance_));
}

void OccupancyGridLocalizerNode::postLoadGraphCallback()
{
  RCLCPP_DEBUG(get_logger(), "[OccupancyGridLocalizerNode] postLoadGraphCallback().");
  getNitrosContext().setParameterFloat64(
    "binary_occupancy_map_loader", "nvidia::isaac::OccupancyGridMap", "cell_size", cell_size_);
  getNitrosContext().setParameterInt32(
    "binary_occupancy_map_loader", "nvidia::isaac::OccupancyGridMap", "threshold",
    occupancy_grid_map_threshold_);
  getNitrosContext().setParameterFloat64(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer", "robot_radius",
    robot_radius_);
  getNitrosContext().setParameterFloat64(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer", "max_beam_error",
    max_beam_error_);
  getNitrosContext().setParameterInt32(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer", "num_beams_gpu",
    num_beams_gpu_);
  getNitrosContext().setParameterInt32(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer", "batch_size",
    batch_size_);
  getNitrosContext().setParameterFloat64(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer",
    "out_of_range_threshold",
    out_of_range_threshold_);
  getNitrosContext().setParameterFloat64(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer",
    "invalid_range_threshold",
    invalid_range_threshold_);
  getNitrosContext().setParameterFloat64(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer",
    "min_scan_fov_degrees",
    min_scan_fov_degrees_);
  getNitrosContext().setParameterBool(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer",
    "use_closest_beam",
    use_closest_beam_);
  getNitrosContext().setParameterFloat64(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer",
    "min_output_error",
    min_output_error_);
  getNitrosContext().setParameterFloat64(
    "global_localization", "nvidia::isaac::localization::GridSearchLocalizer",
    "max_output_error",
    max_output_error_);
  getNitrosContext().setParameterStr(
    "binary_occupancy_map_loader", "nvidia::isaac::ImageLoader", "filename", map_png_path_);
  // decide how much memory to allocate based on how much memory used in
  // FLatScan Message parser source code
  getNitrosContext().setParameterUInt64(
    "flatscan_beams_tensor_copier", "nvidia::gxf::BlockMemoryPool",
    "block_size",
    kFlatScanGxfMsgSize * kMsgBufferSize * sizeof(double) * max_points_);
}

OccupancyGridLocalizerNode::~OccupancyGridLocalizerNode() {}

}  // namespace occupancy_grid_localizer
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(
  nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode)
