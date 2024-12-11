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

cmake_minimum_required(VERSION 3.5)
project(isaac_ros_pointcloud_utils LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Eigen
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# pointcloud_to_flatscan
ament_auto_add_library(pointcloud_to_flatscan SHARED src/pointcloud_to_flatscan_node.cpp)
rclcpp_components_register_nodes(pointcloud_to_flatscan "nvidia::isaac_ros::pointcloud_utils::PointCloudToFlatScanNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::pointcloud_utils::PointCloudToFlatScanNode;$<TARGET_FILE:pointcloud_to_flatscan>\n")
set_target_properties(pointcloud_to_flatscan PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# flatscan_to_laserscan_node
ament_auto_add_library(flatscan_to_laserscan_node SHARED src/flatscan_to_laserscan_node.cpp)
rclcpp_components_register_nodes(flatscan_to_laserscan_node "nvidia::isaac_ros::pointcloud_utils::FlatScantoLaserScanNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::pointcloud_utils::FlatScantoLaserScanNode;$<TARGET_FILE:flatscan_to_laserscan_node>\n")
set_target_properties(flatscan_to_laserscan_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# laserscan_to_flatscan_node
ament_auto_add_library(laserscan_to_flatscan_node SHARED src/laserscan_to_flatscan_node.cpp)
rclcpp_components_register_nodes(laserscan_to_flatscan_node "nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode;$<TARGET_FILE:laserscan_to_flatscan_node>\n")
set_target_properties(laserscan_to_flatscan_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_pointcloud_to_flatscan_pol_test.py)
  add_launch_test(test/isaac_ros_laserscan_to_flatscan_pol_test.py)
  add_launch_test(test/isaac_ros_flatscan_to_laserscan_pol_test.py)
endif()


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config launch)
