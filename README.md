### *** Archieved: As this repo is and will not updated anymore, it is archieved. ***

# Robotic Manipulation Of Deformable Objects with Franka Robot

## Institut Pascal UNIVERSITE CLERMONT AUVERGNE (UCA) 2019

This repository consists the codes from the master thesis of Robotic Manipulation Of Deformable Objects with Franka Robot. Reports that belong to the branches can be reached from "Reports" folder. 

### Joint Position Controller

Joint Position Controller branch is intended for sending both joint position (degree) and velocity (degree/s) commands.

It can be run using: 

'roslaunch franka_robot_controller_pi joint_position_controller.launch'

and commands can be send using:

'rosrun franka_robot_controller_pi teleop_cmd'

0. Set Franka robot to starting pose
1. For sending random position
2. For controlling each joint separately	
3. Read joint positions and velocities from commands.txt


<img title="Joint Position Controller - Trapezoid Velocity Controller" alt="Joint Position Controller - Trapezoid Velocity Controller" src="imgs/trapezoid_screenshot.png" width="600">

### Joint Velocity Controller

Joint Velocity Controller branch is intended for sending joint velocity commands (degree/s) for given time duration (seconds).

It can be run using: 

'roslaunch franka_robot_controller_pi joint_velocity_controller.launch'

and commands can be send using:

'rosrun franka_robot_controller_pi franka_robot_teleop_vel'

0. *Not working for velocity controller* Set Franka robot to starting pose
1. *Not working for velocity controller* For sending random position
2. *Not working for velocity controller* For controlling each joint separately	
3. Read joint positions and velocities from commands.txt (Second column velocity in degrees/s, third column duration in seconds)

### Joint Position Controller - MoveIt! Integration

MoveIt! Controller branch is intended for using MoveIt! user interface over RVIZ to change the pose of the robot using handles. End effector Cartesian pose then will be converted to the joint positions using built in inverse kinematic solver of the MoveIt!

It can be run using: 

'roslaunch franka_robot_controller_pi panda_control_moveit_rviz.launch'

####Note: Joint velocity limits can be set using commands.txt's third column. 
