# Isaac ROS Map Localization

<div align="center"><img alt="Isaac ROS Map Localization Sample Output" src="resources/occupancy_grid_localizer.gif" width="600"/></div>

## Overview

The Isaac ROS Map Localization module contains ROS 2 packages for lidar processing to estimate poses relative to a map. The Occupancy Grid Localizer processes a planar range scan to estimate pose in an occupancy grid map; this occurs in less than 1 second for most maps. This initial pose can be used to bootstrap navigation for mobile robots and has been integrated and tested with Nav2. This can remove the need for upwards of 30 seconds to [manually estimate the position and direction]( https://youtu.be/IrJmuow1r7g?t=1029) of a robot with RViz, for example.

The Occupancy Grid Localizer is designed to work with planar and 3D LIDARs. It uses [Flatscan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg) for input to the GPU-accelerated computation estimating pose. [Flatscan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg) allows for representation of 3D LIDARs, which have variable angular increments between multiple beams.

<div align="center"><img alt="Isaac ROS Map Localization Sample Output" src="resources/isaac_ros_map_localization_nodegraph.png" width="800"/></div>

LaserScan to Flatscan provides conversion from [LaserScan](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg), which by definition has [equal angle increment](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg#L16) between beams, to [Flatscan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg).

PointCloud to FlatScan provides conversion from [pointcloud]( https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg) output from 3D LIDARs to [Flatscan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg).

> **Note**: Localization can be performed mulitiple times during navigation.
<!-- Split blockquote -->
> **Note**: The input FlatScan Message header/frame_id is used to get the transform of the lidar with respect to the robot base_link frame.
<!-- Split blockquote -->
> **Note**: The output `localization_result` is the transform of `base_link` with respect to the frame specified in the `loc_result_frame` (map) ROS parameter.
<!-- Split blockquote -->
> **Note**: Localization can be triggered in one of two ways:
>
> 1. Buffer FlatScan messages received on a topic and trigger the localization using an `std_srvs/Empty` service call.
> 2. Trigger localization every time a FlatScan message is sent to a topic.

Refer to the [Isaac ROS Occupancy Grid Localizer/Usage section](#usage) for more details.

### Isaac ROS NITROS Acceleration

This package is powered by [NVIDIA Isaac Transport for ROS (NITROS)](https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/), which leverages type adaptation and negotiation to optimize message formats and dramatically accelerate communication between participating nodes.

## Performance

The following table summarizes the per-platform performance statistics of sample graphs that use this package, with links included to the full benchmark output. These benchmark configurations are taken from the [Isaac ROS Benchmark](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark#list-of-isaac-ros-benchmarks) collection, based on the [`ros2_benchmark`](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark) framework.

| Sample Graph                                                                                                                                         | Input Size | AGX Orin                                                                                                                                           | Orin NX                                                                                                                                            | Orin Nano 8GB                                                                                                                                        | x86_64 w/ RTX 4060 Ti                                                                                                                                |
| ---------------------------------------------------------------------------------------------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Occupancy Grid Localizer Node](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/scripts//isaac_ros_grid_localizer_node.py) | ~50 sq. m  | [15.9 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-agx_orin.json)<br>65 ms | [7.33 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-orin_nx.json)<br>140 ms | [5.17 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-orin_nano.json)<br>200 ms | [50.1 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_grid_localizer_node-nuc_4060ti.json)<br>20 ms |



## Table of Contents

- [Isaac ROS Map Localization](#isaac-ros-map-localization)
  - [Overview](#overview)
    - [Isaac ROS NITROS Acceleration](#isaac-ros-nitros-acceleration)
  - [Performance](#performance)
  - [Table of Contents](#table-of-contents)
  - [Latest Update](#latest-update)
  - [Supported Platforms](#supported-platforms)
    - [Docker](#docker)
  - [Quickstart](#quickstart)
  - [Next Steps](#next-steps)
    - [Try More Examples](#try-more-examples)
    - [Customize your Dev Environment](#customize-your-dev-environment)
  - [Package Reference](#package-reference)
    - [Isaac ROS Occupancy Grid Localizer](#isaac-ros-occupancy-grid-localizer)
      - [Usage](#usage)
      - [ROS Parameters](#ros-parameters)
      - [ROS Topics Subscribed](#ros-topics-subscribed)
      - [ROS Topics Published](#ros-topics-published)
      - [ROS Services Advertised](#ros-services-advertised)
    - [Isaac ROS PointCloud to FlatScan](#isaac-ros-pointcloud-to-flatscan)
      - [Usage](#usage-1)
      - [ROS Parameters](#ros-parameters-1)
      - [ROS Topics Subscribed](#ros-topics-subscribed-1)
      - [ROS Topics Published](#ros-topics-published-1)
    - [Isaac ROS LaserScan to FlatScan](#isaac-ros-laserscan-to-flatscan)
      - [Usage](#usage-2)
      - [ROS Topics Subscribed](#ros-topics-subscribed-2)
      - [ROS Topics Published](#ros-topics-published-2)
    - [Isaac ROS FlatScan to LaserScan](#isaac-ros-flatscan-to-laserscan)
      - [Usage](#usage-3)
      - [ROS Parameters](#ros-parameters-2)
      - [ROS Topics Subscribed](#ros-topics-subscribed-3)
      - [ROS Topics Published](#ros-topics-published-3)
  - [Troubleshooting](#troubleshooting)
    - [Isaac ROS Troubleshooting](#isaac-ros-troubleshooting)
  - [Updates](#updates)

## Latest Update

2023-05-25: Performance improvements.

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system with an NVIDIA GPU.

| Platform | Hardware                                                                                                                                                                                               | Software                                                                                                           | Notes                                                                                                                                                                                       |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)<br>[Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.1.1](https://developer.nvidia.com/embedded/jetpack)                                                     | For best performance, ensure that the [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | NVIDIA GPU                                                                                                                                                                                             | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/) <br> [CUDA 11.8+](https://developer.nvidia.com/cuda-downloads) |

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on both Jetson and x86_64 platforms.

> **Note**: All Isaac ROS Quickstarts, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.

## Quickstart

1. Set up your development environment by following [these instructions](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).
2. Clone this repository and its dependencies under `~/workspaces/isaac_ros-dev/src`:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_map_localization.git
    ```

3. Pull down a ROS bag of sample data:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_map_localization && \
      git lfs pull -X "" -I "**/rosbags/"
    ```

4. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

5. Inside the container, build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

6. (Optional) Run tests to verify complete and correct installation:

    ```bash
    colcon test --executor sequential
    ```

7. Run ``rviz2``:

    ```bash
    rviz2 -d  src/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer/rviz/quickstart.rviz
    ```

8. Create another terminal in the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

9. Run the launch file to spin up a demo of this package:

    ```bash
    source install/setup.bash && \
    ros2 launch isaac_ros_occupancy_grid_localizer isaac_ros_occupancy_grid_localizer_quickstart.launch.py
    ```

10. Create another terminal in the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

11. Run the rosbag:

    ```bash
    source install/setup.bash && \
    ros2 bag play -l src/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer/data/rosbags/flatscan
    ```

12. Create another terminal in the docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

13. Trigger the localization using a command line service call :

    ```bash
    ros2 service call trigger_grid_search_localization std_srvs/srv/Empty {}
    ```

14. You should see a frame being generated in the map showing the position of the LIDAR.
    <div align="center"><img alt="Isaac ROS Map Localization Sample Output" src="resources/quickstart.png" width="600"/></div>

## Next Steps

### Try More Examples

To continue exploring Isaac ROS MAP localization, check out the following examples:

- [Isaac ROS Occupancy Grid Localizer Nav2 Isaac Sim Tutorial](./docs/isaac-sim-nav2-tutorial.md)

### Customize your Dev Environment

To customize your development environment, refer to [this guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/modify-dockerfile.md).

## Package Reference

### Isaac ROS Occupancy Grid Localizer

#### Usage

```bash
ros2 launch isaac_ros_occupancy_grid_localizer isaac_ros_occupancy_grid_localizer.launch.py
```

> **Note**: Use the `flatscan` topic with the `trigger_grid_search_localization` service to trigger localization using a service.
>
> **Or** publish directly to the `flatscan_localization` topic to trigger localization every time a FlatScan message is received on this topic.
>
> **Do not** publish FlatScan messages to both `flatscan` and `flatscan_localization` topics.

#### ROS Parameters

> **Note**: The ROS param names are the same as the Nav2 [map_server](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_io.cpp#L136) YAML params. This allows to load and pass the same YAML file to both Nav2 and isaac_ros_occupancy_grid_localizer as shown in the [Isaac Sim Launch File](./isaac_ros_occupancy_grid_localizer/launch/isaac_ros_occupancy_grid_localizer_nav2.launch.py)

| ROS Parameter             | Type                  | Default           | Description                                                                                                                                                                                                                                                |
| ------------------------- | --------------------- | ----------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `loc_result_frame`        | `std::string`         | `map`             | frame_id of localization result                                                                                                                                                                                                                            |
| `resolution`              | `double`              | `0.05`            | The meters per pixel of the png map being loaded. This parameter is loaded from the the map YAML file.                                                                                                                                                     |
| `origin`                  | `std::vector<double>` | `[0.0, 0.0, 0.0]` | The origin of the map loaded. Used to transform the output to compensate for the same transform made to the PNG file loaded by the Nav2 [map_server](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_io.cpp#L136).           |
| `occupied_thresh`         | `double`              | `0.65`            | Pixels with occupancy probability greater than this threshold are considered completely occupied. This parameter is loaded from the the map yaml file. <br> Supported values: `[0,1)`                                                                      |
| `image`                   | `std::string`         | `""`              | Name of the PNG file used to load map. This should be in the same directory as the map YAML file specified in `map_yaml_path`. This parameter is loaded from the the map YAML file.                                                                        |
| `map_yaml_path`           | `std::string`         | `""`              | Absolute path to the map yaml file. From which we load the `resolution` and `occupied_thresh`                                                                                                                                                              |
| `max_points`              | `int`                 | `20000`           | Maximum number of points in FlatScan Message that can be received used to pre-allocate GPU memory.                                                                                                                                                         |
| `robot_radius`            | `double`              | `0.25`            | The radius of the robot. This parameter is used to exclude poses which are too close to an obstacle.memory.                                                                                                                                                |
| `min_output_error`        | `double`              | `0.22`            | The minimal output error used to normalize and compute confidence, if output error from best sample smaller or equal to this, the confidence is 1                                                                                                          |
| `max_output_error`        | `double`              | `0.35`            | The max output error from our best sample, if output error larger than this threshold, we conclude localization failed                                                                                                                                     |
| `max_beam_error`          | `double`              | `0.5`             | The maximum beam error used when comparing range scans.                                                                                                                                                                                                    |
| `num_beams_gpu`           | `int`                 | `512`             | The GPU accelerated scan-and-match function can only handle a certain number of beams per range scan. The allowed values are {32, 64, 128, 256, 512}. If the number of beams in the range scan does not match this number a subset of beams will be taken. |
| `batch_size`              | `int`                 | `512`             | This is the number of scans to collect into a batch for the GPU kernel. Choose a value which matches your GPU well.                                                                                                                                        |
| `sample_distance`         | `double`              | `0.1`             | Distance between sample points in meters. The smaller this number, the more sample poses will be considered. This leads to a higher accuracy and lower performance.                                                                                        |
| `out_of_range_threshold`  | `double`              | `100.0`           | Points range larger than this threshold will be marked as out of range and not used.                                                                                                                                                                       |
| `invalid_range_threshold` | `double`              | `0.0`             | Points range smaller than this threshold will be marked as invalid and not used.                                                                                                                                                                           |
| `min_scan_fov_degrees`    | `double`              | `270.0`           | Minimal required scan fov to run the localizer.                                                                                                                                                                                                            |
| `use_closest_beam`        | `bool`                | `true`            | Whether or not pick the closest angle beam in angle bucket, if not pick the average within an angular bucket                                                                                                                                               |


#### ROS Topics Subscribed

| ROS Topic               | Type                                                                                                                                                                      | Description                                                                                                                                                              |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `flatscan`              | [isaac_ros_pointcloud_interfaces::msg::FlatScan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg) | The input FlatScan messages buffer. The last message on this topic will be used as input for localization when the `trigger_grid_search_localization` service is called. |
| `flatscan_localization` | [isaac_ros_pointcloud_interfaces::msg::FlatScan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg) | The topic to trigger localization directly without a buffer. Localization will be triggered every time a FlatScan message is received on this topic.                     |

#### ROS Topics Published

| ROS Topic             | Interface                                                                                                                                              | Description                                                                                                               |
| --------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| `localization_result` | [geometry_msgs::msg::PoseWithCovarianceStamped](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/PoseWithCovarianceStamped.msg) | Pose of the scan data with respect to the map origin, as specified in the first note in the [overview section](#overview) |

#### ROS Services Advertised

| ROS Service                        | Interface                                                                                            | Description                                                                                                |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `trigger_grid_search_localization` | [std_srvs::srv::Empty](https://github.com/ros2/common_interfaces/blob/humble/std_srvs/srv/Empty.srv) | The service to trigger the global localization using the last scan received on the `flatscan` input topic. |

### Isaac ROS PointCloud to FlatScan

#### Usage

```bash
ros2 launch isaac_ros_pointcloud_utils isaac_ros_pointcloud_to_flatscan.launch.py
```

#### ROS Parameters

| ROS Parameter      | Type   | Default  | Description                                                                      |
| ------------------ | ------ | -------- | -------------------------------------------------------------------------------- |
| `threshold_x_axis` | bool   | `false`  | Enable X Axis Threshold                                                          |
| `threshold_y_axis` | bool   | `false`  | Enable Y Axis Threshold                                                          |
| `min_x`            | double | `-1.0`   | Min X axis threshold                                                             |
| `max_x`            | double | `1.0`    | Max X axis threshold                                                             |
| `min_y`            | double | `-1.0`   | Min Y axis threshold                                                             |
| `max_y`            | double | `1.0`    | Max Y axis threshold                                                             |
| `min_z`            | double | `-0.1`   | Min Z axis threshold                                                             |
| `max_z`            | double | `0.1`    | Max Z axis threshold                                                             |
| `max_points`       | int    | `150000` | Maximum number of 3D points in input Point Cloud used to pre allocate GPU memory |

#### ROS Topics Subscribed

| ROS Topic    | Type                                                                                                                   | Description       |
| ------------ | ---------------------------------------------------------------------------------------------------------------------- | ----------------- |
| `pointcloud` | [sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg) | Input Point Cloud |

#### ROS Topics Published

| ROS Topic  | Type                                                                                                                                                                      | Description      |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------- |
| `flatscan` | [isaac_ros_pointcloud_interfaces::msg::FlatScan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg) | Output Flat Scan |

### Isaac ROS LaserScan to FlatScan

#### Usage

```bash
ros2 launch isaac_ros_pointcloud_utils isaac_ros_laserscan_to_flatscan.launch.py
```

#### ROS Topics Subscribed

| ROS Topic | Type                                                                                                               | Description     |
| --------- | ------------------------------------------------------------------------------------------------------------------ | --------------- |
| `scan`    | [sensor_msgs::msg::LaserScan](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg) | Input LaserScan |

#### ROS Topics Published

| ROS Topic  | Type                                                                                                                                                                      | Description      |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------- |
| `flatscan` | [isaac_ros_pointcloud_interfaces::msg::FlatScan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg) | Output Flat Scan |

### Isaac ROS FlatScan to LaserScan

#### Usage

```bash
ros2 launch isaac_ros_pointcloud_utils isaac_ros_flatscan_to_laserscan.launch.py
```

#### ROS Parameters

| ROS Parameter        | Type   | Default      | Description                                                                                                                     |
| -------------------- | ------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| `angle_min`          | double | `0.0`        | The starting angle of the generated LaserScan                                                                                   |
| `angle_max`          | double | `2 * M_PI`   | The ending angle of the generated LaserScan                                                                                     |
| `angle_increment`    | double | `M_PI / 180` | The angle increment per LaserScan reading                                                                                       |
| `time_increment`     | double | `0.0001`     | The time increment per LaserScan reading                                                                                        |
| `max_range_fallback` | double | `200.0`      | If the Max Range of the input FlatScan == 0, then this parameter is used to populate 'Max Range' field of the output LaserScan. |

#### ROS Topics Subscribed

| ROS Topic  | Type                                                                                                                                                                      | Description     |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------- |
| `flatscan` | [isaac_ros_pointcloud_interfaces::msg::FlatScan](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_pointcloud_interfaces/msg/FlatScan.msg) | Input Flat Scan |

#### ROS Topics Published

| ROS Topic | Type                                                                                                               | Description     |
| --------- | ------------------------------------------------------------------------------------------------------------------ | --------------- |
| `scan`    | [sensor_msgs::msg::LaserScan](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg) | Ouput LaserScan |

## Troubleshooting

### Isaac ROS Troubleshooting

For solutions to problems with Isaac ROS, please check [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md).

## Updates

| Date       | Changes                                    |
| ---------- | ------------------------------------------ |
| 2023-05-25 | Performance improvements                   |
| 2023-04-05 | Added `isaac_ros_occupancy_grid_localizer` |
