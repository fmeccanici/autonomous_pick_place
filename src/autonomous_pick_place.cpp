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

	// grasping.open_gripper();
	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);
	// grasping.detach_object();
	//path_planning.home();

	grasping.open_gripper();
	object_detection.detect_objects();
	path_planning.add_objects(object_detection.collision_objects);

	std::vector<std::string> object_ids = {"object_pickup", "object_avoid"};
	std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map = pick_and_place.planning_scene_interface.getObjects(object_ids);

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.resize(2);

	collision_objects[0] = collision_objects_map["object_pickup"];
	
	
	//grasping.detach_object();
	
	
	grasping.determine_goal_pose(1, collision_objects);
	grasping.visualize();

	path_planning.set_goal(grasping.goal_pose);


	path_planning.plan();
	path_planning.execute();
	grasping.close_gripper();


	grasping.attach_object();



/*
	

	// ROS_INFO_STREAM("\t"<<collision_objects_map["object_pickup"];);
	// collision_objects.push_back(collision_objects_map["object_pickup"]);

	ROS_INFO_STREAM("\t" << "CHECK");

	collision_objects[1] = collision_objects_map["object_avoid"];
	
	grasping.determine_goal_pose(1, collision_objects);

	// grasping.determine_goal_pose(0, collision_objects);


	grasping.attach_object();

	path_planning.set_goal(grasping.goal_pose);
	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);
	path_planning.disable_collisions();
	path_planning.plan();
	path_planning.execute();
*/

	// std::vector<std::string> object_ids = {"object_pickup", "object_avoid"};
	// std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map = pick_and_place.planning_scene_interface.getObjects(object_ids);

	// std::vector<moveit_msgs::CollisionObject> collision_objects;
	// collision_objects.resize(2);

	// collision_objects[0] = collision_objects_map["object_pickup"];
	// collision_objects[1] = collision_objects_map["object_avoid"];
	
	
	
	grasping.determine_goal_pose(0, collision_objects);
	//grasping.place();
	grasping.visualize();
	path_planning.set_goal(grasping.goal_pose);
	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);
	path_planning.plan();
	path_planning.execute();

	grasping.open_gripper();	
	grasping.detach_object();

	/*
	path_planning.set_goal(grasping.goal_pose);
	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);
	path_planning.plan();
	path_planning.execute();
	*/
	// grasping.close_gripper();
	
	//grasping.pick();

	//grasping.place();
	// grasping.detach_object();
	// grasping.close_gripper();

	//grasping.detach_object();
	/*
	//grasping.determine_goal_pose(1, object_detection.collision_objects);
	grasping.determine_goal_pose(0, object_detection.collision_objects);

	grasping.visualize();
	//grasping.attach_object();
	path_planning.disable_collisions();


	// grasping.open_gripper();
	path_planning.set_goal(grasping.goal_pose);
	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);
	path_planning.plan();

	
	
	path_planning.execute();
	
	*/

	// sleep(2);
	// grasping.close_gripper();
	// sleep(3);

	// grasping.attach_object();
	
 
	// sleep(5);
	/*
	grasping.determine_goal_pose(0, object_detection.collision_objects);
	grasping.visualize();

	path_planning.set_goal(grasping.goal_pose);	
	path_planning.plan();
	

	path_planning.execute();
	sleep(3);
	grasping.detach_object();

	
	grasping.open_gripper();
	*/

	
	
	return EXIT_SUCCESS;
}

