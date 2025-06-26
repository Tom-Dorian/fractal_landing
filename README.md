# aruco_fractal_tracker

A ROS2 package for detecting and estimating the pose of ArUco fractal markers.

## Overview

`aruco_fractal_tracker` is a ROS2 node that subscribes to camera images, detects ArUco fractal markers using the [aruco](https://www.uco.es/investiga/grupos/ava/node/26) library, and publishes the estimated marker pose and annotated images. It also broadcasts the marker pose as a TF transform.

## Features

- Detects ArUco fractal markers in camera images
- Publishes annotated images with detected markers
- Publishes marker pose as a `geometry_msgs/PoseStamped`
- Broadcasts marker pose as a TF transform

## Dependencies

- ROS2 (rclcpp, rclcpp_components)
- [aruco](https://www.uco.es/investiga/grupos/ava/node/26)
- OpenCV
- cv_bridge
- geometry_msgs
- sensor_msgs
- tf2, tf2_ros, tf2_geometry_msgs
- dji_msdk_ros

## Building

Clone this repository into your ROS2 workspace `src` directory and build using `colcon`:

```sh
cd ~/Phd_ws
colcon build --packages-select aruco_fractal_tracker
```

## Usage

Source your workspace and run the node:

```sh
source install/setup.bash
ros2 run aruco_fractal_tracker aruco_fractal_tracker
```

The node subscribes to `/camera/image_raw` and publishes:

- Annotated images on `Aruco_image`
- Marker pose on `/arucoPose`
- TF transform from `simple_drone/bottom_cam_link` to `marker_frame`

## Node Parameters

Currently, camera parameters and marker size are hardcoded in the source. Adjust them in [`src/aruco_fractal_tracker_node.cpp`](src/aruco_fractal_tracker_node.cpp) as needed.

## File Structure

- [`include/aruco_fractal_tracker/aruco_fractal_tracker_node.hpp`](include/aruco_fractal_tracker/aruco_fractal_tracker_node.hpp): Node header
- [`src/aruco_fractal_tracker_node.cpp`](src/aruco_fractal_tracker_node.cpp): Node implementation
- [`src/aruco_fractal_tracker_node_main.cpp`](src/aruco_fractal_tracker_node_main.cpp): Node entry point
- [`CMakeLists.txt`](CMakeLists.txt): Build instructions
- [`package.xml`](package.xml): ROS2 package manifest

## License

This project is licensed under the GPL-3.0 License. See the [LICENSE](https://www.gnu.org/licenses/gpl-3.0.html) file for details.

## Author

Dmitry Anikin (<dmitry.anikin@proton.me>)