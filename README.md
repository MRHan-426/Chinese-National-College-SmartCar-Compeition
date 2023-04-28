<p align="center">
  <img src="https://github.com/MRHan-426/Chinese-National-College-SmartCar-Compeition/blob/master/.assets/2.png" alt="image" width="66%" height="auto">
</p>

![status](https://img.shields.io/badge/status-archived-EB1923)
![last modified](https://img.shields.io/badge/last%20modified-04%2F27%2F2023-EB1923)

This repository is used to archive the code of the 16th and 17th Chinese National College SmartCar Competition(NCSC). 

Our team stood out from 127 teams from 98 colleges and won the **championship** in the 16th competition.

The next year, our junior schoolmates won the national first prize in the 17th competition.

## **1.Introduction**
As we just started to get familiar with ROS, we used and learned from many existing ros packages. 

The project we worked on is more inclined to connect and integrate ros packages to complete the competition. 

Since the code depends on hardware, it cannot be run directly. Here is a brief introduction to the functions of each part.


├── [ucar_cam](https://github.com/ros-drivers/usb_cam) : We get RGB pictures from monocular camera.

├── [ucar_map](https://github.com/ros-planning/navigation/tree/melodic-devel/map_server) : Publish prior map in the navigation process.

├── [ucar_nav](https://github.com/ros-planning/navigation) : We use A* algorithm as global planner, and TEB algorithm as local planner. We modify params to reach higher speed and avoid obstacles.

├── [ydlidar](https://github.com/YDLIDAR/ydlidar_ros_driver) : Get data from Lidar.

├── [iris_lama](https://github.com/iris-ua/iris_lama_ros) : A lightweight package for Localization using odom and Lidar.

├── [robot_localization](https://github.com/cra-ros-pkg/robot_localization) : We use EKF to fuse wheel odom and IMU, in order to increase localization accuracy, which is crucial to navigation.

├── [yolov4-for-darknet_ros](https://github.com/Tossy0423/yolov4-for-darknet_ros) : We collect data to train and use yolov4-tiny for real-time recogition on Jetson Nano. 

├── logic_module : The logic that connects the entire competition process. Judge the status of the mobile robot and make decisions.

├── xf_mic_asr_offline : We use the offline speech recognition function of iFlytek to wake up and control the mobile robot through voice commands.

├── ucar_controller : Control the motion of the mobile robot with Mecanum wheel model, serving as a bridge between the upper and lower computers.

└── CMakeLists.txt

## 2.Examples




