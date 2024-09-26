# Isaac ROS Map Localization

NVIDIA-accelerated Map localization.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_map_localization/occupancy_grid_localizer.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_map_localization/occupancy_grid_localizer.gif/" width="600px"/></a></div>

## Overview

The [Isaac ROS Map Localization](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_map_localization) module contains ROS 2 packages for lidar
processing to estimate poses relative to a map. The Occupancy Grid
Localizer processes a planar range scan to estimate pose in an occupancy
grid map; this occurs in less than 1 second for most maps. This initial
pose can be used to bootstrap navigation for mobile robots and has been
integrated and tested with Nav2. This can remove the need for upwards of
30 seconds to [manually estimate the position and
direction](https://youtu.be/IrJmuow1r7g?t=1029) of a robot with RViz,
for example.

The Occupancy Grid Localizer is designed to work with planar and 3D
LIDARs. It uses
[Flatscan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg)
for input to the GPU-accelerated computation estimating pose.
[Flatscan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg)
allows for representation of 3D LIDARs, which have variable angular
increments between multiple beams.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_map_localization/isaac_ros_map_localization_nodegraph.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_map_localization/isaac_ros_map_localization_nodegraph.png/" width="800px"/></a></div>

LaserScan to Flatscan provides conversion from
[LaserScan](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg),
which by definition has [equal angle
increment](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg#L16)
between beams, to
[Flatscan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg).

PointCloud to FlatScan provides conversion from
[pointcloud](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg)
output from 3D LIDARs to
[Flatscan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg).

> [!Note]
> Localization can be performed multiple times during
> navigation.

> [!Note]
> The input FlatScan Message header/frame_id is
> used to get the transform of the lidar with respect to the robot
> base_link frame.

> [!Note]
> The output `localization_result` is the
> transform of `base_link` with respect to the frame specified in the
> `loc_result_frame` (map) ROS parameter.

> [!Note]
> Localization can
> be triggered in one of two ways:

> 1. Buffer FlatScan messages received on a topic and trigger the
>    localization using an `std_srvs/Empty` service call.
> 2. Trigger localization every time a FlatScan message is sent to a
>    topic.

Refer to the [Isaac ROS Occupancy Grid Localizer/Usage section](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer/index.html) for more details.

## Isaac ROS NITROS Acceleration

This package is powered by [NVIDIA Isaac Transport for ROS (NITROS)](https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/), which leverages type adaptation and negotiation to optimize message formats and dramatically accelerate communication between participating nodes.

## Performance

| Sample Graph<br/><br/>                                                                                                                                                                                                  | Input Size<br/><br/>          | AGX Orin<br/><br/>                                                                                                                                                     | Orin NX<br/><br/>                                                                                                                                                      | Orin Nano 8GB<br/><br/>                                                                                                                                                  | x86_64 w/ RTX 4060 Ti<br/><br/>                                                                                                                                          | x86_64 w/ RTX 4090<br/><br/>                                                                                                                                           |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Occupancy Grid Localizer Node](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/benchmarks/isaac_ros_occupancy_grid_localizer_benchmark/scripts/isaac_ros_grid_localizer_node.py)<br/><br/><br/><br/> | ~50 sq. m<br/><br/><br/><br/> | [19.5 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-agx_orin.json)<br/><br/><br/>57 ms @ 30Hz<br/><br/> | [8.34 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-orin_nx.json)<br/><br/><br/>130 ms @ 30Hz<br/><br/> | [5.75 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-orin_nano.json)<br/><br/><br/>190 ms @ 30Hz<br/><br/> | [50.1 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-nuc_4060ti.json)<br/><br/><br/>21 ms @ 30Hz<br/><br/> | [50.1 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-x86_4090.json)<br/><br/><br/>12 ms @ 30Hz<br/><br/> |

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_occupancy_grid_localizer`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer/index.html#quickstart)
  * [Try More Examples](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer/index.html#try-more-examples)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer/index.html#api)
* [`isaac_ros_pointcloud_utils`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_pointcloud_utils/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_pointcloud_utils/index.html#overview)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_map_localization/isaac_ros_pointcloud_utils/index.html#api)

## Latest

Update 2024-09-26: Update for Isaac ROS 3.1
