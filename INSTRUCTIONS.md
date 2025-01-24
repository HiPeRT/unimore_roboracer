# Instructions
This document covers the steps required to set up and run the unimore_roboracer repository.

## Table of Contents
1. [Building the Stack](#building-the-stack)
2. [Running the Stack](#running-the-stack)
3. [Demonstration](#demonstration)
4. [Troubleshooting](#troubleshooting)

## Building the Stack

### Prerequisites
This guide assumes you are working on an Ubuntu 20.04 distribution.

#### ROS2
The ROS2 framework is used as the main data transfer and abstraction middleware for node-to-node communication. The project is developed and tested targeting the ROS2 `foxy` distribution. Follow the [ROS2 Documentation](https://docs.ros.org/en/foxy/Installation.html) to install and set up ROS2 in your system.

#### ROS2 dependencies
Our stack relies on multiple additional ROS2 dependencies, can install them via apt with
```bash
sudo apt install libyaml-cpp-dev ros-foxy-nav2-map-server ros-foxy-ackermann-msgs ros-foxy-tf2 ros-foxy-tf2-eigen ros-foxy-tf2-ros ros-foxy-nav2-msgs python3-colcon-common-extensions ros-foxy-gps-msgs ros-foxy-xacro ros-foxy-joint-state-publisher
```

#### Additional non-ROS dependencies
In order to perform optimized linear algebra and image processing tasks, we rely on `Eigen3` and `OpenCV`. For custom configuration files, we rely on the `yaml` library. You can install this dependencies them with the following:
```bash
sudo apt install libeigen3-dev libopencv-dev libyaml-cpp-dev
```

#### Optional dependencies
In order to better visualize operational data, debug messages or perform data analytics on this software, you may want to install [`rviz2`](https://github.com/ros2/rviz), [`foxglove-studio`](https://foxglove.dev/) and/or [`plotjuggler`](https://github.com/facontidavide/PlotJuggler).


### Build
After installing all of the required packages, you can clone the repository.

```bash
git clone https://github.com/hipert/unimore_roboracer.git
cd unimore_roboracer
```

And finally build the stack with colcon

```bash
colcon build
```

**Warning!** This will build all of the packages of the repository. Memory-constrained devices with less than 4GB of RAM may not be able to build the entire workspace due to the high memory footprint of the compilation. Consider using `--parallel-workers N` to limit the number of parallel compilation workers or `--packages-up-to <pkg_1> <pkg_2>` to reduce the number of packages to build in the current run of `colcon build`.

**Friendly advice:** Consider building the stack with `--symlink-install` to install all files as symbolic links. This means that editing any file in the workspace will instantly reflect the changes on the installed files (this obviously does not apply to sources that need to be compiled). This makes all changes done on `config` and `launch` files immediately work without re-running `colcon build`.

## Running the Stack
To launch the autonomous navigation stack, you need to source the ROS2 general setup script
```bash
source /opt/ros/foxy/setup.bash
```

Source the workspace-specific setup script
```bash
# from the unimore-roboracer directory
source install/setup.bash
```

To launch a basic trajectory-following demo, use `kitt_pp.launch.xml`.

```bash
ros2 launch kitt pp_global.launch.xml
```

After a brief initialization phase, the autonomous stack is ready to run.

## Troubleshooting
If you encounter any issues during the build or run process, you can refer to the [ROS2 Troubleshooting Guide](https://docs.ros.org/en/foxy/Troubleshooting.html) for common solutions.

For issues specific to this project, check the repository's [issue tracker](https://github.com/hipert/unimore_roboracer/issues) for existing issues
  - If you don't find any issue describing your problem open a new issue specifying your running environment and the issues encountered.
