# README #
This repository is copied and modified from (https://github.com/ShaoTziYang/ML3-14)
##Overview
The whole system consists out of slave-robots and a central computer building a multi-robot SLAM system. This part is the slave-robot part.

##Objective
This project was aimed to create a software framework that enables Simultaneous Localization and Mapping(A.K.A. SLAM) with multiple robots on a median level laptop. The entire framework is established on Robot Operating System(A.K.A. ROS, http://www.ros.org/).

##Run System
To run the whole multi-robot SLAM system, read the instructions in the repository (http://bitbucket.org/schneida/centralcomputer_hkustproject)

##Run slave-robot
Read README_STARTUP.txt

##System Flow  

###Hardware  
The framework aimed to realized multiple robot SLAM on a median level laptop. The hardware structure of a robot is: a median level laptop, a turltebot(http://www.turtlebot.com/), and a calibrated camera. The setup of the turtlebot system on ROS can be find on the tutorial section of official page(http://wiki.ros.org/Robots/TurtleBot), and also the calibration of camera(http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). In our project, we tried omni-directional camera and fish eye camera since it is very easily for LSD-SLAM to loose track of the images once there is insufficient overlapping. Using cameras with larger field of view can solve it problem.

###Sofware  
The software system is seperated implemented on the turtlebot robots and the central computer:

####Turtlebot  
LSD-SLAM is running on this end of the system, pointclouds(sensor_msgs::PointCloud2), pose( geometry_msgs/PoseStamped, and keyframe(sensor_msgs::Image). Which keyframe is a single capture of the camera stream, it is chosen when there is significant angle or distance difference toward the previous keyframe. Angle and distance differences are estimtaed by the minimization of the photometric error, *for detailed explanation of the keyframe system please read the paper of LSD-SLAM.* All three messages are transmitted toward the central computer via Wi-Fi by running the same ROS across multiple machines(tutorial http://wiki.ros.org/ROS/Tutorials/MultipleMachines). The code realization of the above steps is in src/pc_fetcher.cpp. Point cloud data obtained by LSD-SLAM are down sampled with random sampling and sampled with raduisoutlier removal to obtained better results, the implmentaion is done via the PCL library in src/pc_filter.cpp.

####Central Computer
On this side of the system, multiple datas obtained by multiple instance of robots might be sent to the central computer. The merge of the local point clouds are done by the keyframe labels. With the trained vocabularies, central computer will be able to tell when two keyframes are from around the same area, which we call it image matches. When we got the image matches from FAB-MAP, we use the pose obtained by LSD-SLAM as the initial guess and we perform iterative closest point on the point clouds obtained by two key frames from two robots. After the iterative closest point estimation, we are able to get the rotation and translation of the point clouds in two keyframes, apply it to the local map and we can get the global map.

##Related Libraries
This framework mainly consists of two SLAM methods, a monocular density SLAM called *LSD-SLAM[1]*, and a visual-based topological SLAM called *FAB-MAP[2]*. Point cloud obtained are filtered with the help of implementation in the Point Cloud Library, *PCL[3]*.  
*[1] LSD-SLAM: Large-Scale Direct Monocular SLAM, J. Engel, T. Schöps, D. Cremers, ECCV '14*  
*(link http://vision.in.tum.de/research/lsdslam)*  
*[2]  FAB-MAP,Accelerated Appearance-Only SLAM,M. Cummins, P. Newman, ICRA '08*  
*(link http://www.robots.ox.ac.uk/~mjc/Software.htm)*  
*[3] Point Cloud Library (link http://www.pointclouds.org/)*  

##Authors
Tzi Yang Shao and Daniel Schneider (schneida@student.ethz.ch)