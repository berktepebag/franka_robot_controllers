#ifndef FOLLOW_INTERACTIVE_MARKER_H_
#define FOLLOW_INTERACTIVE_MARKER_H_

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <sstream>
#include <string>

class Franka_robot{

private:
	std::vector<std::string> joint_names;
	std::vector<double> joint_goal_positions;
	std::vector<double> joint_velocities;
	std::vector<double> joint_efforts;

	
public:
	int const FRANKA_ROBOT_JOINT_NUMBER = 7;
	std::vector<double> const  start_pose = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};


	void setJointNames(){
		//Sets default names
		for (int i = 0; i < FRANKA_ROBOT_JOINT_NUMBER; ++i)
		{
			std::stringstream ss;
			ss << i+1;
			std::string str = "panda_joint"+ss.str();
			joint_names.push_back(str);
			//std::cout << str << std::endl;
		}
	}
	void setJointName(int joint_id,std::string jointName){
		joint_names[joint_id] = jointName;
	}	
	std::vector<std::string> getJointNames() const{
		return joint_names;
	}
	void setJointStartPosition(){
		//Set joint goal positions to starting position
		joint_goal_positions = start_pose;
	}
	void setJointGoalPosition(int joint_id, float joint_goal_position){
		joint_goal_positions[joint_id] = joint_goal_position;
	} 	
	double getJointGoalPosition(int joint_id){ 
		return joint_goal_positions[joint_id];
	}
	std::vector<double> getJointGoalPositions() const{
		return joint_goal_positions;
	}
	void setJointVelocitiesZero(){
		joint_velocities = {0,0,0,0,0,0,0};
	}	
	void setJointVelocitiesOne(){
		joint_velocities = {deg2rad(1),deg2rad(1),deg2rad(1),deg2rad(1),deg2rad(1),deg2rad(1),deg2rad(1),};
	}
	void setJointVelocity(int joint_id, float joint_velocity){
		joint_velocities[joint_id] = joint_velocity;
	}
	std::vector<double> getJointVelocities() const{
		return joint_velocities;
	}
	void setJointEfforts(){
		joint_efforts = {1,1,1,1,1,1,1};
	}
	void setJointEffort(int joint_id, float joint_effort){
		joint_efforts[joint_id] = joint_effort;
	}
	double deg2rad(double degree){
		return degree * M_PI /180;
	}
};
#endif