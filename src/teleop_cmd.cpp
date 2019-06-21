#include <teleop_cmd.h>

int main(int argc, char **argv)
{
	Franka_robot franka_robot;
	franka_robot.setJointNames(); // Sets the joint names to the default names.
	franka_robot.setJointStartPosition(); // Sets the joint positions to start position
	
	// Initialize the franka_robot_teleop node
	ros::init(argc,	argv, "franka_robot_teleop");

	ros::NodeHandle nh;

	ros::Publisher teleop_cmd_pub = nh.advertise<sensor_msgs::JointState>("franka_robot/teleop_cmd", 1000);
	ros::Rate loop_rate(50);

	sensor_msgs::JointState joint_state_msg;
	joint_state_msg.name = franka_robot.getJointNames();

	franka_robot.setJointStartPosition();
	joint_state_msg.position = franka_robot.getJointGoalPositions();	

	//Set joint velocities to zero in order to prevent unexpected movements at the beginning.
	franka_robot.setJointVelocitiesZero();	

	//nh.getParam("panda_joi")

	double add = 0.25;
	while(ros::ok()){		

		franka_robot.setJointVelocitiesOne();

		int joint_id;
		std::cout << "Enter new joint number (1-7) to be moved or 0 to set Franka robot to start position." << std::endl;
		std::cin >> joint_id;

		if (joint_id>7)
		{
			int joint_speed;
			std::cout << "Enter new joint speed." << std::endl;
			std::cin >> joint_speed;

			if (joint_id==8) add = 2*0.25;
			else if (joint_id==9) add = 2*-0.25;
			
			std::vector<double> trial_pose{0+add, -M_PI_4+add, 0+add, -3 * M_PI_4-add/2, 0+add, M_PI_2+add, M_PI_4+add};

			int counter = 0;
			for(double pose : trial_pose){
				std::cout << "joint["<<counter+1<<"]: " <<pose << std::endl;
				franka_robot.setJointGoalPosition(counter,pose);
				franka_robot.setJointVelocity(counter,joint_speed);
				counter++;
			}			
		}
		else if (joint_id == 0)
		{
			franka_robot.setJointStartPosition();
		}
		else if(joint_id>0 & joint_id <=7){

			double newJointPos = 0;
			std::cout << "Enter new joint (exact) position (- or +) in degree's for joint " << joint_id << std::endl;

			std::cin >> newJointPos;

			franka_robot.setJointGoalPosition(joint_id-1, franka_robot.deg2rad(newJointPos));

			double newJointVelocity;
			std::cout << "Enter new joint velocity in degree/sec for joint " << joint_id << std::endl;

			std::cin >> newJointVelocity;
			franka_robot.setJointVelocity(joint_id-1,newJointVelocity);
			
		}
		else{
			std::cout << "Joint id must be between 1-7 (included)! Please fix your command!" << std::endl;
			continue;
		}

		joint_state_msg.velocity = franka_robot.getJointVelocities();
		joint_state_msg.position = franka_robot.getJointGoalPositions();

		teleop_cmd_pub.publish(joint_state_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}