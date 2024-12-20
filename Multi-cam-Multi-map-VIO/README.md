# Multi-cam-Multi-map-VIO

## Introduction

This repository contains the source code of the algorithm **Multi-cam-Multi-map-VIO** (Multi-camera Multiple Maps Based VIO), which consistently fuses multiple maps into VIO to improve its performance. This algorithm is based on [C-MIMB-VIO](https://github.com/zhuqingzhang/C-MIMB-VIO) and extends it to support multi-camera setups.

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

### 2. Launch File Configuration
In `use_example.launch`, update the following parameters:

1. Map storage directory:
  ```xml
  <arg name="map_save_path" default="/path/to/map/storage/" />

2. Matcher URL:
  ```xml
  DetectAndMatch_URL: "http://your.matcher.url:6000/process_images"
  Note: Must match your online matcher configuration from step 2

3. Image storage directory:
```xml
  DetectAndMatch_img_save_path: "/path/to/image/storage/"

3.change url in launch file
            DetectAndMatch_URL: "http://10.192.4.95:6000/process_images"
please change it to your URL, it must be same to your online matcher's result. (step 2)

4.change your image save paths,
you should create a document to save your images,
            DetectAndMatch_img_save_path: "/home/cxhu/new/test/"
please change it to your path.


##usage
to run the examle file ,you should do these 4 step in order
1.open first terminal ,run your ros master
roscore
2.open second terminal, run online matcher
cd /home/cxhu/Documents/online_matcher/result/    (please to change it to your matcher's file path)
./main ./config/loc_online.yaml
3.open third terminal, run launch file
roslaunch ov_msckf use_example.launch
4.open forth, run our bag
cd /home/cxhu/Documents   (please to change it to your bag's file path)
rosbag play bag2_2020-11-12-15-28-30.bag --start 7
