# Multi-cam-Multi-map-VIO

## Introduction

This repository contains the source code of the algorithm **Multi-cam-Multi-map-VIO**, which consistently fuses multiple maps into VIO to improve its performance. This algorithm is based on [C-MIMB-VIO](https://github.com/zhuqingzhang/C-MIMB-VIO) and extends it to support multi-camera setups.

## Installation
This repository is currently only support ROS1. All the dependencies is the as those in Open-VINS. You can follow the guidance of [Open-VINS Installation](https://docs.openvins.com/gs-installing.html) to install the dependencies.

## System Requirements

### Dependencies
- ROS1 (currently the only supported ROS version)
- All dependencies from Open-VINS ([Installation Guide](https://docs.openvins.com/gs-installing.html))

### Recommended Environment
- Ubuntu 20.04
- ROS Noetic
- CUDA/cuDNN 9.1.0
- OpenGV (specific version available at: https://github.com/slinkle/2-Entity-RANSAC)
- OpenCV 4.0

## Configuration Steps

### 1. Map Database Setup
To use the provided pre-built map:
1. Locate the configuration files in `./result/config`:
   - `loc_online.yaml`
   - `matcher.yaml`
   - `retrieval.yaml`
2. Update the database path in these files:
   ```yaml
   database_dir: "/path/to/your/database"
   database_name: "qsdjt"
   ```

### 2. Launch File Configuration
In `use_example.launch`, update the following parameters:

1. Map storage directory:
   ```xml
   <arg name="map_save_path" default="/path/to/map/storage/" />
   ```

2. Matcher URL:
   ```xml
   DetectAndMatch_URL: "http://your.matcher.url:6000/process_images"
   ```
   Note: Must match your online matcher configuration from step 2

3. Temporary image storage directory:
   ```xml
   DetectAndMatch_img_save_path: "/path/to/image/storage/"
   ```

## Usage Instructions

Execute the following steps in order:

1. Start ROS master:
   ```bash
   roscore
   ```

2. Launch online matcher:
   ```bash
   cd /path/to/online_matcher/result/
   ./main ./config/loc_online.yaml
   ```

3. Launch the main node:
   ```bash
   roslaunch ov_msckf use_example.launch
   ```

4. Play the example rosbag:
   ```bash
   cd /path/to/bagfile/
   rosbag play bag2_2020-11-12-15-28-30.bag --start 7
   ```
