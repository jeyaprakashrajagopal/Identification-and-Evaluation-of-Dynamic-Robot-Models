=====================================================
Package name :: youbot_moveit_config
Src folder   :: /src/youbot_moveit_config.cpp
Dependencies :: moveit_ros_planning_interface, catkin
Input	     :: youbot.urdf
Output	     :: Collision free joint angles
Author 	     :: Jeyaprakash Rajagopal
=====================================================

Install moveit for ROS indigo:
------------------------------
1. To install copy and paste the following script on terminal 

sudo apt-get install ros-indigo-moveit 

2. Setup your environment

source ~/catkin_ws/devel/setup.bash

To run the simulation in Rviz:
---------------------------------
1. Launch the robot.launch file using the command

roslaunch youbot_moveit_config robot.launch

2. Launch the demo.launch file using the command

roslaunch youbot_moveit_config demo.launch

3. Execute the program using the command

rosrun youbot_moveit_config youbot_moveit_config
