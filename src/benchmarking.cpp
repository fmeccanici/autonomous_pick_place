#include "benchmarking.h"

Benchmarking::Benchmarking()
: move_group("arm_torso")
{
	//std::string planning_group = ;
	this->spinner.start();
}

Benchmarking::~Benchmarking()
{
	this->spinner.stop();
}

autonomous_pick_place::benchmarking Benchmarking::set_parameters()
{
	autonomous_pick_place::benchmarking benchmarking_msg;


	// benchmarking_msg.motion_planner = move_group.getPlannerId();
	
	benchmarking_msg.end_effector_pose = move_group.getCurrentPose();

	// benchmarking_msg.planning_time = my_plan.planning_time_;
	// benchmarking_msg.robot_trajectory = my_plan.trajectory_;

	return benchmarking_msg;
}


void Benchmarking::subscriber_callback(const sensor_msgs::JointState &joint_states)
{
	
	ROS_INFO_STREAM("\t CHECK1");

	auto benchmarking_msg = set_parameters();
	benchmarking_msg.joint_states = joint_states;

	benchmarking_state_publisher.publish(benchmarking_msg);

}
