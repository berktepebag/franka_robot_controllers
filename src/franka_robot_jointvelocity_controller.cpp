#include <franka_robot_jointposition_controller.h>

#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <math.h>
//Added
#include <hardware_interface/joint_command_interface.h>


namespace franka_robot_controllers{
	
	void JointVelocityController::frankaRobotTeleopCallback(const sensor_msgs::JointState::ConstPtr& msg) {

		if (msg->name.size() != msg->position.size())
		{
			ROS_ERROR("Name and Position numbers are not equal! Check panda_arm_teleop topic.");
			return;
		}        

		this->setJointPositionGoals(msg->position);
		this->setJointSpeedLimits(msg->velocity); 		

		// Convert received angle joint velocities to radian
		joint_goal_velocities = {speed_mult[0],speed_mult[1],speed_mult[2],speed_mult[3]			,speed_mult[4],speed_mult[5],speed_mult[6]};
	}

	bool JointVelocityController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh){

		ROS_INFO_STREAM(" ******* \n Starting Franka Robot Joint Velocity Controller \n *******");

		initial_pose_.resize(7);
		// Set speed multipliers to zero in order to prevent robot move without command at the beginning
		speed_mult = {0,0,0,0,0,0,0};		
		// Set joint commands size equal to joint number
		joint_commands.resize(7);
		// Set Velocity limits size equal to joint number
		joint_goal_velocities.resize(7);
		current_joint_goal_velocities.resize(7);
		//Each joint is acting differently with given commands. These variables make sure they are working without jerk.

		// Original acceleration limits from Franka-Emika. They are divided by d, in order to make controller smooth. Second joint affected by the movement of the other joints which is relatively smaller than others.
		double d = 15;
		joint_accelerations = {15/d,10/10,10/d,12.5/12.5,15/d,20/d,20/d}; // rad/s^-2

		// Subscribe to the teleop_cmd
		teleop_cmd_sub = nh.subscribe("/franka_robot/teleop_cmd", 1000, &JointVelocityController::frankaRobotTeleopCallback, this);

		velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();

		if (velocity_joint_interface_ == nullptr)
		{
			ROS_ERROR("JointVelocityController: Could not get joint velocity interface from hardware!");
			return false;
		}

		std::vector<std::string> joint_names;
		if (!nh.getParam("joint_names",joint_names))
		{
			ROS_ERROR("JointVelocityController: Could not get joint names.");
		}
		if (joint_names.size()!=7)
		{
			ROS_ERROR_STREAM("JointVelocityController: Did not recieve 7 joints, Franka robot is 7 DOF robot!");
			return false;
		}

		velocity_joint_handles_.resize(7);
		for (int i = 0; i < 7; ++i)
		{
			try{
				velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
			}									
			catch(const hardware_interface::HardwareInterfaceException& e){
				ROS_ERROR_STREAM("JointVelocityController: Could not get joint handles: " << e.what());
				return false;
			}
		}

		auto state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
		if (state_interface == nullptr)
		{
			ROS_ERROR("JointVelocityController: Could not get state interface from hardware!");
			return false;
		}

		bool safety_check = false;
		if (safety_check)
		{
			std::array<double, 7> q_start{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
			try	{

				auto state_handle = state_interface->getHandle("panda_robot");

				for (int i = 0; i < q_start.size(); ++i)
				{
					if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1)
					{
						ROS_ERROR_STREAM("Robot is not in the starting position. This is only for safety of the robot. If you do not want to use this function just turn safety_check to false. Run `roslaunch franka_example_controllers move_to_start.launch robot_ip:=<robot-ip> first to move Franka robot to starting position.");
						return false;
					}
				}
			}catch(const hardware_interface::HardwareInterfaceException& e) {
				ROS_ERROR_STREAM(
					"JointVelocityController: Exception getting state handle: " << e.what());
				return false;
			}
		}
		return true;
	}

	void JointVelocityController::starting(const ros::Time&) {			
		elapsed_time_ = ros::Duration(0.0);
	}

	void JointVelocityController::update(const ros::Time&, const ros::Duration& period){
		
		//Get teleop_cmd values from related topic.
		ros::spinOnce();
		//Total time passed since controller started in ROS::time
		elapsed_time_ += period;
		//Total time passed since controller started converted to the double.
		double seconds_passed = elapsed_time_.toSec();	
	}

}
// Implementation name_of_your_controller_package::NameOfYourControllerClass,
PLUGINLIB_EXPORT_CLASS(franka_robot_controllers::JointVelocityController,
	controller_interface::ControllerBase)
