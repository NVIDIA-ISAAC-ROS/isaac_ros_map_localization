# Isaac ROS Occupancy Grid Localizer Nav2 Isaac Sim Tutorial

<div align="center"><img alt="Isaac ROS  Occupancy Grid Localizer Expected Output" src="../resources/nav2_isaac_sim.png" width="600px"/></div>

## Overview

This tutorial describes how to integrate the Isaac ROS Occupancy Grid Localizer with a simulated robot running in Isaac Sim with Nav2.

Last validated with [Isaac Sim 2022.1.0](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/release_notes.html#id10).

## Tutorial Walkthrough

1. Complete the [Quickstart section](../README.md#quickstart) in the main README.
2. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

3. Inside the container, build and source the workspace:  

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

4. Install and launch Isaac Sim following the steps in the [Isaac ROS Isaac Sim Setup Guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/isaac-sim-sil-setup.md)
5. Open up the Isaac ROS Common USD scene (using the *Content* tab) located at:
    
    ```text
    http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd
    ```
   And wait for it to load completely.
6. Press **Play** to start publishing data from the Isaac Sim application.

7. Run the launch file and you should see a nav2 window pop up.

    ```bash
    ros2 launch isaac_ros_occupancy_grid_localizer isaac_ros_occupancy_grid_localizer_nav2.launch.py
    ```

8. Open a new terminal using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

9. Trigger the localization service using the command line interface:

    ```bash
    ros2 service call /trigger_grid_search_localization std_srvs/srv/Empty {}
    ```

10. You should see the laser scan in rviz. This happens when the transform between map and carter_lidar frame is available. The robot is now localized. You can give a position setpoint using the `2D Nav Goal` button as shown below. You can relocalize again at any point by running the `ros2 service call` command above.

<div align="center"><img alt="Isaac ROS  Occupancy Grid Localizer Expected Output" src="../resources/nav2_isaac_sim.gif" width="600px"/></div>
