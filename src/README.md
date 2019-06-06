# Digital Futures ROS

## Overview

Digital Futures ROS is a computer vision based localization system for the mobile robots being implemented at the Digital Futures Workshop 2019. It also offers a Ros-bridge interface for controlling the robots' movements through Grasshopper environment. It is intended for the Raspberry Pi Zero W running Raspbian Stretch.

**Keywords:** opencv, ros, localization, mobile robotics, rpi

### License

The source code is released under (N/A)

**Author: Nicolas KK (Tech Support) <br />
Affiliation: Institute for Computational Design and Construction (ICD) <br />
Maintainer: Nicolas KK, nkal@mica.edu**

The Digital Futures ROS package has been tested under [ROS] Kinetic and Raspbian Stretch. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation


### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [OpenCV 3.4] (real-time computer vision)
		http://www.codebind.com/cpp-tutorial/install-opencv-ubuntu-cpp/
		https://www.pyimagesearch.com/2015/12/14/installing-opencv-on-your-raspberry-pi-zero/
- [Raspicam] https://www.uco.es/investiga/grupos/ava/node/40


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/t3ch-support/Digital-Futures-ROS.git
	cd ../
	catkin_make


## Usage


Run the main node with

	roslaunch digital_futures marker_pose_estimator.launch

Generate marker images with

	roslaunch digital_futures marker_generator.launch

Calibrate camera with

	roslaunch digital_futures camera_calibrator.launch

## Config files

Config file

* **calibration.yaml** Camera calibration parameters for pose estimation


## Launch files

* **marker_pose_estimator.launch:** launches marker detection node and broadcasts robot id with position

* **marker_generator.launch:** generates a markers (or board) in JPG format for use in calibration of camera and marker detection

* **camera_calibrator.launch:** launches interactive camera calibration window, press c to capture images of charuco board; press esc to generate calibration file to use for pose estimation


## Nodes

### marker_pose_estimator

Checks image capture for aruco markers and broadcasts marker id, pose of robot, and robot id


#### Subscribed Topics

None

#### Published Topics

* **`/digital_futures/robot_id/marker`** ([std_msgs/Int64.msg])

	Marker ID

* **`/digital_futures/robot_id/pose`** ([geometry_msgs/PoseStamped.msg])

	Array of markers as position and quaternion orientations


#### Parameters


* **`camera_calibration`** (string)

	Path to camera calibration config file.

* **`dictionary_id`** (float)

	Which aruco dictionary to use for detection (DICT_4X4_50=0, DICT_4X4_100=1... see launch file)

* **`corner_refinement_method`** (int)

	0: None, 1: Subpixel, 2:contour, 3: AprilTag 2

* **`marker_length`** (float)

	Length in meters of single physical aruco marker.

* **`charuco_board`** (bool)

    True if detecting board, false if detecting single markers.

* **`board_marker_length`** (float)

    Length in meters of single physical aruco marker on board.

* **`board_square_length`** (float)

    Length in meters of physical charuco squares.

* **`board_squaresX`** (int)

    Number of squares in charuco board's width.

* **`board_squaresY`** (int)

    Number of squares in charuco board's height.


### charuco_board_generator

Used for generating marker images (JPG) for camera calibration and/or pose estimation


#### Parameters

* **`board_or_single`** (int)

	Choose whether to generate a charuco board or a set of single markers (board = 0, singles = 1)

* **`single_count`** (int)

	Number of single markers to generate

* **`single_size`** (int)

	Size of marker to generate in pixels

* **`dictionary_id`** (float)

	Which aruco dictionary to use for generation (DICT_4X4_50=0, DICT_4X4_100=1... see launch file)

* **`board_marker_length`** (int)

    Length in pixels of single aruco marker on board.

* **`board_square_length`** (int)

    Length in pixels of charuco squares.

* **`board_squaresX`** (int)

    Number of squares in charuco board's width.

* **`board_squaresY`** (int)

    Number of squares in charuco board's height.

* **`border_bits`** (int)

    Number of bits in marker borders.

* **`output_path`** (string)

    Path to save JPG.




### camera_calibrator

Interactive camera calibration tool


#### Subscribed Topics

None


#### Parameters


* **`dictionary_id`** (float)

	Which aruco dictionary to use for calibration (DICT_4X4_50=0, DICT_4X4_100=1... see launch file)

* **`board_marker_length`** (float)

    Length in meters of single aruco marker on board.

* **`board_square_length`** (float)

    Length in meters of charuco squares.

* **`board_squaresX`** (int)

    Number of squares in charuco board's width.

* **`board_squaresY`** (int)

    Number of squares in charuco board's height.


* **`output_path`** (string)

    Path to save calibration file.




## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kyjanond/timberbot/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[OpenCV 3]: https://opencv.org/
[video_stream_opencv]: https://github.com/ros-drivers/video_stream_opencv