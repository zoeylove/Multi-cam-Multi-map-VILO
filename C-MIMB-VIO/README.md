# C-MIMB-VIO

## Introduction

This repository contains the source code of the algorithm C-MIMB-VIO (Consistent Multiple Isolated Maps Based VIO), which consistently fuses the multiple maps into VIO to improve the performance of VIO. This algorithm is based on the open-sourced framework [Open-VINS](https://github.com/rpng/open_vins).



The online map-matching module is not provided in this repository yet, Only the back-end algorithm is provided. However, the folder 'matches' provides map matching information between the query sequences and the maps on  different [used dataset](#dataset) . 

Interested readers can refer to the attached paper in the "docs" folder for the details.



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


