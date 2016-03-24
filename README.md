# ackslam_simulation
desktop and simulation packages for autonomous cars.

## Overview
* **vrep_common** and **vrep_plugins** are ros packages that should go to *catkin_ws/src* folder.Or you can directly clone this repo to your *catkin_ws/src*.  
* **vrep_scenes** contains vrep scene files that are ready to use.

##Version Information

For now, **vrep_plugin** and **vrep_common** have been tested in the following envronment.

* VREP version : V-REP PRO EDU, version 3.2.2

* ROS version: ROS indigo

* Operating System version: Ubuntu 14.04 LTS

## Install
1. Install V-REP.  
  * Version 3.2.2 is recommended. [Here](http://www.coppeliarobotics.com/previousVersionDownloads.html) is the download page link of V-REP. Choose **V-REP PRO EDU, Linux 32 or 64**.

  * Extract the downloaded file and put it wherever you want.    
2. Compile the plugin lib for V-REP in ROS.  
  * git clone this repo to your *catkin_ws/src*. Run command below
  ```
  catkin_make or catkin build
  ```
  * copy the lib file **/catkin_ws/devel/lib/libv_repExtRos.so** to the V-REP root folder.

3. Run V-REP with ROS
  * Keep it in mind that run **roscore** before starting V-REP.  
  ```
  roscore
  ```

  * Run V-REP. In the V-REP root folder,run command  
  ```
  ./vrep.sh  or sh vrep.sh
  ```
4. Load Ray-with-ros-enabled-sensors scene file and run.

5. Run **ackslam** node.
  ```
  rosrun ackslam ackslam_node
  ```
6. Control the car to move around using your keyboard.

## Usage
There is one scene available now.
* **Ray-with-ros-enabled-sensors** is a small traffic road scene for full size autonomous cars.(It is fast!)

What kind of data can you get from V-REP for now?
* Imu
* GPS with noise
* Odometry
* Steering Angle
* LaserScan or PointCloud
* Vision
* tf
* ...

What can you control for now?
* Steering angle
* Longitudinal speed
* Drive type
