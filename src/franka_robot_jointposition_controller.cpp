#include <franka_robot_jointposition_controller.h>

#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <math.h>

namespace franka_robot_controllers{
	
	void JointPositionController::frankaRobotTeleopCallback(const sensor_msgs::JointState::ConstPtr& msg) {

		if (msg->name.size() != msg->position.size())
		{
			ROS_ERROR("Name and Position numbers are not equal! Check panda_arm_teleop topic.");
			return;
		}        

		this->setJointPositionGoals(msg->position);
		this->setJointSpeedLimits(msg->velocity); 		

		// Convert received angle joint velocities to radian
		joint_velocity_limits = {speed_mult[0]*M_PI/180,speed_mult[1]*M_PI/180,speed_mult[2]*M_PI/180,speed_mult[3]*M_PI/180,speed_mult[4]*M_PI/180,speed_mult[5]*M_PI/180,speed_mult[6]*M_PI/180};
	}

	bool JointPositionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh){

		ROS_INFO_STREAM(" ******* \n Starting Franka Robot Joint Position Controller \n *******");

		initial_pose_.resize(7);
		// Set speed multipliers to zero in order to prevent robot move without command at the beginning
		speed_mult = {0,0,0,0,0,0,0};		
		// Set joint commands size equal to joint number
		joint_commands.resize(7);
		// Set Velocity limits size equal to joint number
		joint_velocity_limits.resize(7);
		current_joint_velocity_limits.resize(7);
		//Each joint is acting differently with given commands. These variables make sure they are working without jerk.
		joint_accelerations = {25,5,25,15,25,25,20};

		// Subscribe to the teleop_cmd
		teleop_cmd_sub = nh.subscribe("/franka_robot/teleop_cmd", 1000, &JointPositionController::frankaRobotTeleopCallback, this);



		position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();

		if (position_joint_interface_ == nullptr)
		{
			ROS_ERROR("Could not get joint position interface from hardware!");
			return false;
		}

		std::vector<std::string> joint_names;
		if (!nh.getParam("joint_names",joint_names))
		{
			ROS_ERROR("Could not get joint names.");
		}
		if (joint_names.size()!=7)
		{
			ROS_ERROR_STREAM("Did not recieve 7 joints, Franka robot is 7 DOF robot!");
			return false;
		}

		position_joint_handles_.resize(7);
		for (int i = 0; i < 7; ++i)
		{
			try{
				position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
			}									
			catch(const hardware_interface::HardwareInterfaceException& e){
				ROS_ERROR_STREAM("Could not get joint handles: " << e.what());
				return false;
			}
		}

		bool safety_check = false;

		if (safety_check)
		{
			std::array<double, 7> q_start{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};

			for (int i = 0; i < q_start.size(); ++i)
			{
				if (std::abs(position_joint_handles_[i].getPosition() - q_start[i] > 0.1))
				{

					ROS_ERROR_STREAM("Robot is not in the starting position. This is only for safety of the robot. If you do not want to use this function just turn safety_check to false. Run `roslaunch franka_example_controllers move_to_start.launch robot_ip:=<robot-ip> first to move Franka robot to starting position.");
					return false;
				}
			}
		}
		return true;
	}

	void JointPositionController::starting(const ros::Time&) {

	//Set the first command to current joint positions
		for (size_t i = 0; i < 7; ++i) {
			joint_commands[i] = position_joint_handles_[i].getPosition();
		}
		joint_position_goals = joint_commands;
		
		elapsed_time_ = ros::Duration(0.0);

		// This prevents shakes at the beginning
		//joint_position_goals = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
	}

	void JointPositionController::update(const ros::Time&, const ros::Duration& period){
		
		//Get teleop_cmd values from related topic.
		ros::spinOnce();
		//Total time passed since controller started in ROS::time
		elapsed_time_ += period;
		//Total time passed since controller started converted to the double.
		double seconds_passed = elapsed_time_.toSec();	
		//Set distance to goal point to 0 in order to check with each new loop. 	
		double distance_to_goal_point = 0;

		//For each joint check if they are at goal position or move them to the goal position.
		for (int joint_id = 0; joint_id < 7; ++joint_id)
		{
			//Distance between goal position and joint's current position. 
			distance_to_goal_point = joint_position_goals[joint_id] - position_joint_handles_[joint_id].getPosition();

			//Since joints are turning both -/+ sides, we need it as a multiplier. 
			int direction = std::signbit(distance_to_goal_point)==1?-1:1;

			//If distance_to_goal_point is more than this value keep joints working. Less than 0.001 causes problems becareful with oscillations.
			double goal_distance_check_value = 0.001;	
			//For joint speed less than 10 deg/s set slowing angle directly 1/20
			double slowing_distance = joint_velocity_limits[joint_id]/20;

			double jVelDeg = rad2deg(joint_velocity_limits[joint_id]);

			if (std::fabs(distance_to_goal_point)>goal_distance_check_value || seconds_passed <= 0.001) // Seconds_passed prevents oscillations at the first call of the controller. DO NOT DELETE!
			{				
				//If joint speed is >10 deg/s apply 5+ slowing angle to prevent oscillations. Later decrease it to original value to keep precision.
				if (joint_velocity_limits[joint_id]>deg2rad(10) & current_joint_velocity_limits[joint_id] > deg2rad(5)){

					slowing_distance = deg2rad(5+int(jVelDeg/10));
				} 		
				// If absolute distance to goal point is less than slowing distance start slowing.
				if (std::fabs(distance_to_goal_point) < slowing_distance)
				{
					if(debugging) std::cout << "slowing down" << std::endl;
					current_joint_velocity_limits[joint_id] /= 1.005;
				}
				//If joint's speed is less than goal velocity keep accelerating.		
				else if (std::fabs(joint_velocity_limits[joint_id]-std::fabs(position_joint_handles_[joint_id].getVelocity()))>1*M_PI/180)
				{
					if(debugging) std::cout << "accelerating" << std::endl;
					current_joint_velocity_limits[joint_id] += joint_accelerations[joint_id]*M_PI/180*period.toSec();
				}
				//Reached goal speed, keep it.
				else{
					current_joint_velocity_limits[joint_id] = joint_velocity_limits[joint_id];
				}

				if(debugging) std::cout << "current speed: " << position_joint_handles_[joint_id].getVelocity()*180/M_PI << std::endl;

				//Final command to be sent to the Franka robot.
				joint_commands[joint_id] += (direction)*current_joint_velocity_limits[joint_id]*period.toSec();
				//Send the command.
				position_joint_handles_[joint_id].setCommand(joint_commands[joint_id]);
			}
		}
	}
}
// Implementation name_of_your_controller_package::NameOfYourControllerClass,
PLUGINLIB_EXPORT_CLASS(franka_robot_controllers::JointPositionController,
	controller_interface::ControllerBase)
























		//std::cout << "period: " << (1/period.toSec()) << std::endl;
/*

				std::cout << "distance_to_goal_point " << joint_id<<": " << distance_to_goal_point << std::endl;
				std::cout << "speed_mult[" << joint_id<<"]: " << speed_mult[joint_id] << std::endl;
				std::cout << "joint_velocity_limits[" << joint_id<<"]: " << joint_velocity_limits[joint_id] << std::endl;

				std::cout << "joint [" << joint_id<<"] distance_to_goal_point: " << distance_to_goal_point<<std::endl;
				std::cout << "joint_commands[" << joint_id<<"]: " << joint_commands[joint_id]<<std::endl;
				std::cout << "speed_mult[" << joint_id<<"]: " << speed_mult[joint_id]<<std::endl;
				std::cout << "joint_velocity_limits[" << joint_id<<"]: " << joint_velocity_limits[joint_id]<<std::endl;
				*/