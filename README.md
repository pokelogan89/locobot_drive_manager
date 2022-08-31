# locobot_experimental_drive_manager
### Internal Drive Manager for Locobots

## Overview
This is the ROS package for the drive and joint manager for the WX-250, designed to operate in a closed loop
The user gives the robot an instruction to either drive or rotate, along with a speed and a goal (speed in m/s, goal in meters/degrees)
The robot uses the odometry data as well as the bumper data to safely move the robot to the goal
The robot, in order to ensure the safety of the arm, will raise the arm before any movement, complete the movement, and then lower the arm

## Main Branch
The main branch uses a list of instructions to sequentially send to the robot of the form [["drive",3,.2],["rotate",90,.4]]4

To use the main branch, start both the kobuki and the WX-250
Power on the robot, the battery, and the NUC
Ensure the computer is connected to the external facing network for the HIVE Lab, and ssh into the robot

  roslaunch kobuki_node minimal.launch --screen
  roslaunch xsarm_control xsarm_control.launch "robot_model":="mobile_wx250s" "use_rviz":=false
  roslaunch locobot_experimental_drive_manager drive_manager.launch

The robot will then execute all the commands as written in /src/locobot_experimental_drive_manager/drive_manager.py

## Haptics Communication Branch
The haptics branch instead opens a topic upon which to receive commands of the form "command:drive:3:.2", "command:rotate:90:.4", or "command:stop"
This topic is /haptics/command/drive

To use the haptics branch, start both the kobuki and the WX-250
Power on the robot, the battery, and the NUC
Ensure the computer is connected to the external facing network for the HIVE Lab, and ssh into the robot

  roslaunch locobot_experimental_drive_manager drive_manager.launch "robot_model":="mobile_wx250s"
  
From here, commands can be published over the topic 
Published movement commands will execute until the goal is reached or until the "command:stop" is seen
If there is another command issued while the current one is being executed, the robot will execute the new command after finishing the old one

## Folder Structure
### /locobot_experimental_drive_manager
This contains all of the python source code used in the package
drive_manager.py is the main logic for the drive manager
drive_manager_objcet.py is source for the class DriveManager and all of its functions
  This is where ROS communication and initialization takes place
 
joint_manager.py is the main logic for the joint_manager
joint_manager_object.py is source for the class JointManager and all of its function
  This is where ROS communication and initialization takes place
  
### /nodes
This contains all of the nodes used, which is only the drive manager and the joint manager

drive_manager_node.py
joint_manager_node.py

There is no operational logic contained within these nodes, only starting the function created in the respective source code

### /launch

This contains the launch files used, which is only the drive manager

drive_manager.launch 

For the main branch, this only initializes the joint manager and the drive manager to the capacities that they need, and does not start the robot at all
For the haptics branch, the kobuki_node and xsarm_control launch files are both launched with the proper configurations so that usability is more accessible
