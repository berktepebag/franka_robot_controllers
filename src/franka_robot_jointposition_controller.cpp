#include <franka_robot_jointposition_controller.h>

#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>

namespace franka_robot_controllers{
	
	void JointPositionController::frankaRobotTeleopCallback(const sensor_msgs::JointState::ConstPtr& msg) {

		if (msg->name.size() != msg->position.size())
		{
			ROS_ERROR("Name and Position numbers are not equal! Check panda_arm_teleop topic.");
			return;
		}        

		this->setJointPositionGoals(msg->position);
		this->setJointSpeedLimits(msg->velocity); 		

		// Change joint velocities
		joint_velocity_limits = {speed_mult[0]*M_PI/180,speed_mult[1]*M_PI/180,speed_mult[2]*M_PI/180,speed_mult[3]*M_PI/180,speed_mult[4]*M_PI/180,speed_mult[5]*M_PI/180,speed_mult[6]*M_PI/180};
	}

	bool JointPositionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh){

		ROS_INFO_STREAM(" ******* \n Starting Franka Robot Joint Position Controller \n *******");

		initial_pose_.resize(7);
		// Set speed multipliers to zero in order to prevent robot move without command at the beginning
		speed_mult = {1,0,0,0,0,0,0};		
		// Set joint commands size equal to joint number
		joint_commands.resize(7);
		// Set Velocity limits size equal to joint number
		joint_velocity_limits.resize(7);

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
		
		elapsed_time_ = ros::Duration(0.0);

		// This prevents shakes at the beginning
		joint_position_goals = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
	}

	void JointPositionController::update(const ros::Time&, const ros::Duration& period){

		ros::spinOnce();

		elapsed_time_ += period;
		double seconds_passed = elapsed_time_.toSec();		
		double error = 0;

		for (int joint_id = 0; joint_id < 7; ++joint_id)
		{
			error = joint_position_goals[joint_id] - position_joint_handles_[joint_id].getPosition();

			if (std::fabs(error)>0.01 || seconds_passed <= 0.001){

				int direction = std::signbit(error)==1?-1:1;

				joint_commands[joint_id] += (direction)*joint_velocity_limits[joint_id]*period.toSec();
				/*
				std::cout << "joint [" << joint_id<<"] error: " << error<<std::endl;
				std::cout << "joint_commands[" << joint_id<<"]: " << joint_commands[joint_id]<<std::endl;
				std::cout << "speed_mult[" << joint_id<<"]: " << speed_mult[joint_id]<<std::endl;
				std::cout << "joint_velocity_limits[" << joint_id<<"]: " << joint_velocity_limits[joint_id]<<std::endl;
				*/
				
				position_joint_handles_[joint_id].setCommand(joint_commands[joint_id]);
			}
		}
	}
}

// Implementation name_of_your_controller_package::NameOfYourControllerClass,

PLUGINLIB_EXPORT_CLASS(franka_robot_controllers::JointPositionController,
	controller_interface::ControllerBase)