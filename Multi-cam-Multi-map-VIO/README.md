# Multi-cam-Multi-map-VIO

## Introduction

This repository contains the source code for the **Multi-cam-Multi-map-VIO** algorithm, which extends the **C-MIMB-VIO** (Consistent Multiple Isolated Maps Based VIO) framework by incorporating a panoramic multi-camera system. This enhancement aims to improve the performance of Visual-Inertial Odometry (VIO) by fusing multiple maps from different cameras into a single cohesive VIO system.

The algorithm is based on the open-source framework [Open-VINS](https://github.com/rpng/open_vins), and the VIO implementation builds on its backend, while the map-matching module for online operations is not yet included in this repository. However, map matching information between query sequences and maps is available in the folder 'matches' for reference.

This repository contains the source code of the algorithm C-MIMB-VIO (Consistent Multiple Isolated Maps Based VIO), which consistently fuses the multiple maps into VIO to improve the performance of VIO. This algorithm is based on the open-sourced framework [Open-VINS](https://github.com/rpng/open_vins).


## Installation
This repository is currently only support ROS1. All the dependencies is the as those in Open-VINS. You can follow the guidance of [Open-VINS Installation](https://docs.openvins.com/gs-installing.html) to install the dependencies.


## Usage

```
$mkdir -p catkin_ws/src
$cd catkin_ws/src
$git clone https://github.com/zhuqingzhang/C-MIMB-VIO.git
$catkin_make
$source devel/setup.bash
$roslaunch ov_msckf pgeneva_ros_eth_multimap.launch
```

Then the user should open another terminal to play dataset rosbag. FOr example, to play MH03 rosbag of EuRoC:

```
$rosbag play MH_03_medium.bag  -s 20
```

Then, the user should see the following running interface:

![image](https://github.com/zhuqingzhang/C-MIMB-VIO/blob/main/docs/demo.png)




## <span id="dataset">Used Dataset</span>

In the paper, two datasets are used.

- [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

- [KAIST](https://sites.google.com/view/complex-urban-dataset)


