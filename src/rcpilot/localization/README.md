# Localization module

## Overview
The `Localization` module of our UAV Companion Computer Library is dedicated to the implementation and management of the Visual Inertial Odometry (VIO) system for our UAV. This module contains the necessary code, calibration files, and system management tools required for effective odometry.

## Features

### Visual Inertial Odometry System
The core of this module is the code related to the VIO system. This system uses visual and inertial data to estimate the pose and velocity of the UAV over time, which is crucial for navigation and control.

### Algorithms
We utilize several state-of-the-art algorithms to achieve accurate and reliable odometry. These include:

- [OpenVINS](https://docs.openvins.com/): An open-source platform for research in visual-inertial estimation.
- [VinsFUSION](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion): A robust and accurate VIO system that fuses several sensor data.
- [ZED](https://www.stereolabs.com/): A stereo camera based VIO system.

Each algorithm has its own dedicated files within this module.

### Calibration Files
Proper calibration is crucial for the performance of VIO systems. This module contains the necessary calibration files for our sensors and algorithms.

### Odometry Management System
The Odometry Management System is a real-time management tool designed to unify and manage the various components of our UAV's odometry. Its primary purpose is to ensure seamless integration and efficient operation of the Visual Inertial Odometry (VIO) system, camera wrappers, and communication protocols such as MAVROS and MAVLink.
