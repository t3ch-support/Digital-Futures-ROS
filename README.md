# HoloCV

## Overview

HoloCV is a package meant to provide OpenCV localization functionality to the HoloLens in Unity. This is done using the interface provided by kyjanond's Unity-ROSBridge-UWP

**Keywords:** opencv, hololens, localization

### License

The source code is released under (N/A)

**Author: Nicolas KK (Tech Support) <br />
Affiliation: Institute for Computational Design and Construction (ICD) <br />
Maintainer: Nicolas KK, nkal@mica.edu**

The HoloCV package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation


### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [OpenCV 3] (real-time computer vision)
		http://www.codebind.com/cpp-tutorial/install-opencv-ubuntu-cpp/
- [video_stream_opencv] (optional: for simulating local video stream)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/kyjanond/timberbot.git
	cd ../
	catkin_make


## Usage


Run the main node with

	roslaunch holo_cv marker_pose_estimator.launch

Generate a Charuco Board with

	roslaunch holo_cv charuco_board_generator.launch

Calibrate camera with

	roslaunch holo_cv camera_calibrator.launch

## Config files

Config file

* **calibration.yaml** Camera calibration parameters for pose estimation


## Launch files

* **marker_pose_estimator.launch:** launches video_stream_opencv/camera.launch, rviz for marker visualization, and marker detection window

* **charuco_board_generator.launch:** generates a Charuco Board in JPG format for use in calibration of camera and marker detection

* **camera_calibrator.launch:** launches interactive camera calibration window, press c to capture images of charuco board; press esc to generate calibration file to use for pose estimation


## Nodes

### marker_pose_estimator

Receives sensor_msgs/Image, converts it to cv::Mat and localizes markers


#### Subscribed Topics

* **`/webcam/image_raw`** ([sensor_msgs/Image])

	Image stream from camera to be localized


#### Published Topics

* **`/holocv/output_video`** ([sensor_msgs/Image])

	Processed image with markers detected

* **`/holocv/markers`** ([geometry_msgs/PoseArray])

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

Used for generating a Charuco Board (JPG) of markers for camera calibration and/or pose estimation


#### Parameters



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

* **`/webcam/image_raw`** ([sensor_msgs/Image])

	Image stream from camera to be calibrated


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