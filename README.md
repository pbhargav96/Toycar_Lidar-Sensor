Project Done By

Prateek Bhargava - 116947992
Amoghavarsha Prasanna - 116952910
Sai Praveen Bhamidipati - 117023640

Phase1 - LIDAR and IMU Sensor Attached
Phase2 - Errors while implementing the controllers

Video of Phase 1 with sensors attached is added.

##Dependencies
Ubuntu 16.04/18.04
ROS Kinetic
Gazebo
ydlidar_ros package

## How to run the file

Clone the repository in your ROS workspace. Type:
  1) cd ~/<ROS_Workspace>/src
  2) git clone https://github.com/pbhargav96/Toycar_Lidar-Sensor.git

Launch the world by typing below commands in terminal:
  1) cd ~/<ROS_Workspace>            ## ROS_Workspace is the location of your catkin workspace
  1) source /devel/setup.bash
  2) catkin_make
  3) roslaunch toycar gazebo.launch 


Stop the execution by using Ctrl + Z
