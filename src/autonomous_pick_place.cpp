#include "autonomous_pick_place.h"
#include "grasping.h"
#include "object_detection.h"
#include "path_planning.h"

AutonomousPickPlace::AutonomousPickPlace()
{}

AutonomousPickPlace::~AutonomousPickPlace()
{}

int main(int argc, char** argv)
{	

	ros::init(argc, argv, "autonomous_pick_place_node");



	AutonomousPickPlace pick_and_place;

	ObjectDetection object_detection;
	Grasping grasping;
	PathPlanning path_planning;

	double planning_time, vel_scale_factor;
	std::string reference_frame, planner_id;
	geometry_msgs::PoseStamped goal_pose;


	planning_time = 5.0;
	vel_scale_factor = 1.0;
	reference_frame = "base_footprint";
	planner_id = "SBLkConfigDefault";	

	// grasping.detach_object();
	 // grasping.close_gripper();
	 // sleep(5);
	 // grasping.open_gripper();



	
	object_detection.detect_objects();
	path_planning.add_objects(object_detection.collision_objects);

	
	// grasping.pick();

	// grasping.place();
	// grasping.detach_object();

	grasping.determine_goal_pose(1, object_detection.collision_objects);
	grasping.visualize();
	
	path_planning.set_goal(grasping.goal_pose);
	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);
	path_planning.plan();


	path_planning.execute();
	
	sleep(2);
	grasping.close_gripper();
	sleep(3);
	grasping.attach_object();
	/*

	// sleep(5);
	grasping.determine_goal_pose(0, object_detection.collision_objects);
	grasping.visualize();

	path_planning.set_goal(grasping.goal_pose);	
	path_planning.plan();
	

	path_planning.execute();
	sleep(3);
	grasping.detach_object();

	// path_planning.visualize();
	

	// sleep(5);
	
	grasping.open_gripper();

	

	*/

	return EXIT_SUCCESS;
}

