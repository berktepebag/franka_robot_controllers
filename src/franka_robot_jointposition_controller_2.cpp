				double time_needed_to_stop_with_current_speed_and_acc = position_joint_handles_[joint_id].getVelocity() / joint_accelerations[joint_id]; 
				double distance_needed_to_stop_with_current_speed = joint_accelerations[joint_id]*time_needed_to_stop_with_current_speed_and_acc*time_needed_to_stop_with_current_speed_and_acc/2;

				if ((joint_position_goals[joint_id] - (direction *  distance_needed_to_stop_with_current_speed))  )
				{
					std::cout << "distance_needed_to_stop_with_current_speed: " << distance_needed_to_stop_with_current_speed << std::endl;
					current_joint_velocity_limits[joint_id] -= joint_accelerations[joint_id]*period.toSec();
				}
				else if (joint_velocity_limits[joint_id] - current_joint_velocity_limits[joint_id] < deg2rad(1) )
				{
					std::cout << "acceleration" << std::endl;
					current_joint_velocity_limits[joint_id] += joint_accelerations[joint_id]*period.toSec();
				}else{
					current_joint_velocity_limits[joint_id] = joint_velocity_limits[joint_id];
				}

				joint_commands[joint_id] += direction * current_joint_velocity_limits[joint_id
					]*period.toSec();