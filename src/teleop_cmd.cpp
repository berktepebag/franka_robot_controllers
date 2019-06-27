#include <teleop_cmd.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>

void Franka_robot::sendRandomCommand(Franka_robot &franka_robot){	

	bool sendCommand = 0;
	std::cout << "Enter 1 to send random position!"<< std::endl;
	std::cin >> sendCommand;
	if (sendCommand){			
		int joint_count = 0;
		for(std::string joint_name : franka_robot.getJointNames()){
			int direction = 0;

			if (rand()%2)	direction = -1;

			double rand_pos = rand() % 30 * direction;
			double rand_speed = rand() % 15;	

			if (joint_count == 3)
			{
				if (direction != -1)
				{
					rand_pos *= -1;
				}
			}
			franka_robot.setJointGoalPosition(joint_count, franka_robot.start_pose[joint_count]+franka_robot.deg2rad(
				rand_pos));

			
			franka_robot.setJointVelocity(joint_count,franka_robot.deg2rad(rand_speed));

			joint_count++;
		}
	}

	std::vector<double> joint_pos = franka_robot.getJointGoalPositions();
	std::vector<double> joint_spd = franka_robot.getJointVelocities();

	for (int i = 0; i < 7; ++i)
	{
		std::cout << "Joint " << i << ": Pos: " << joint_pos[i]*180/M_PI << " Spd: " << joint_spd[i] << std::endl; 	
	}
};

void Franka_robot::sendOneJointCommand(Franka_robot &franka_robot){

	int joint_id;
	std::cout << "Enter new joint number (1-7) to be moved or 0 to set Franka robot to start position." << std::endl;
	std::cin >> joint_id;

	if(joint_id>0 & joint_id <=7){

		double newJointPos = 0;
		std::cout << "Enter new joint (exact) position (- or +) in degree's for joint " << joint_id << std::endl;

		std::cin >> newJointPos;

		franka_robot.setJointGoalPosition(joint_id-1, franka_robot.deg2rad(newJointPos));

		double newJointVelocity;
		std::cout << "Enter new joint velocity in degree/sec for joint " << joint_id << std::endl;

		std::cin >> newJointVelocity;
		franka_robot.setJointVelocity(joint_id-1, franka_robot.deg2rad(newJointVelocity));

	}
	else{
		std::cout << "Joint id must be between 1-7 (included)! Please fix your command!" << std::endl;		
	}
}

void Franka_robot::readFromFileCommand(Franka_robot &franka_robot){

	std::ifstream infile("/home/student/catkin_ws/src/franka_ros/franka_robot_controller_pi/src/commands.txt");
	if (infile.is_open())
	{	
		int id;
		float pos, spd;
		char c;

		while(infile >> id >> c >> pos >> c >> spd && c == ','){
			std::cout << "id: " << id << " pos: " << pos << " spd: " << spd << std::endl;
			franka_robot.setJointGoalPosition(id,franka_robot.deg2rad(pos));
			franka_robot.setJointVelocity(id,franka_robot.deg2rad(spd));
		}
	}
	else{
		std::cout << "could not open file" << std::endl;
	}

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

	//Set joint velocities to zero in order to prevent unexpected movements at the beginning.
	franka_robot.setJointVelocitiesZero();	

	//nh.getParam("panda_joi")	

	srand(time(NULL));
	
	while(ros::ok()){		

		franka_robot.setJointVelocitiesOne();

		std::cout << "Enter\n0 for start pose\n1 for random positions or\n2 for controlling each joint 1by1\n3 for reading command from file" << std::endl;
		int choice = -1;
		std::cin >> choice;
		if (choice==0)
		{
			franka_robot.setJointStartPosition();
		}
		else if (choice==1)
		{
			franka_robot.sendRandomCommand(franka_robot);
		}else if (choice==2)
		{
			franka_robot.sendOneJointCommand(franka_robot);
		}else if (choice==3)
		{
			franka_robot.readFromFileCommand(franka_robot);
		}else{
			std::cout << "Wrong entry! Try again." << std::endl;
		}		

		joint_state_msg.velocity = franka_robot.getJointVelocities();
		joint_state_msg.position = franka_robot.getJointGoalPositions();

		teleop_cmd_pub.publish(joint_state_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}