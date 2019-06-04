#include <teleop_cmd.h>

double deg2rad(double degree){
	return degree * M_PI /180;
}

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

	franka_robot.setJointVelocitiesZero();
	

	while(ros::ok()){
		
		//for (int i = 0; i < 1; ++i)
		//{
			int joint_id;
			std::cout << "Enter new joint number (1-7) to be moved." << std::endl;
			std::cin >> joint_id;

			if (joint_id == 0)
			{
				franka_robot.setJointStartPosition();
			}
			else{

			double newJointPos = 0;
			std::cout << "Enter new joint position to add (- or +) in degree's for joint " << joint_id+1 << " or enter 999 to pass." << std::endl;

			std::cin >> newJointPos;
			//if (newJointPos==-999) continue;
			//else
			//{	
				franka_robot.setJointGoalPosition(joint_id-1, deg2rad(newJointPos));

				double newJointVelocity;
				std::cout << "Enter new joint velocity in degree/sec for joint " << joint_id+1 << " or enter -1 to pass." << std::endl;

				std::cin >> newJointVelocity;
				franka_robot.setJointVelocity(joint_id-1,newJointVelocity);
			//}
			}

		//}			
		//std::cout << "hello"<< std::endl;

		joint_state_msg.velocity = franka_robot.getJointVelocities();
		joint_state_msg.position = franka_robot.getJointGoalPositions();

		teleop_cmd_pub.publish(joint_state_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}