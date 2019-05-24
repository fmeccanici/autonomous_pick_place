#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/String.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>
#include <vector>
#include <map>

#include "autonomous_pick_place.h"
#include "grasping.h"
#include "object_detection.h"
#include "path_planning.h"
#include "benchmarking.h"

#include <iostream>
#include <fstream>


using namespace std;

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "autonomous_pick_place_node");
	ros::NodeHandle nh;


	AutonomousPickPlace pick_and_place;
	ObjectDetection object_detection;
	Grasping grasping;
	PathPlanning path_planning;

	double planning_time, vel_scale_factor;
	std::string reference_frame, planner_id;
	geometry_msgs::PoseStamped goal_pose;

	// Boolean value to indicate if we want to pick (1) of place (0)
	bool pick;

	planning_time = 5.0;
	vel_scale_factor = 1.0;
	reference_frame = "base_footprint";
	planner_id = "ProjEST";	

	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);

	// double offset1 = 0.25;

	// Code for picking up objects
	pick = 1;
	grasping.detach_object();
	grasping.open_gripper();

	// Bring end effector to home position (for testing multiple planners)
	path_planning.home();

	// Start marker detection and detect defined objects
	object_detection.detect_objects();

	// Add objects to path planning scene
	path_planning.add_objects(object_detection.collision_objects);

	// Get collision objects from path planning scene, so when vision of the markers is obscured still the objects can be used. This is only useful if
	// objects did not change their position
	std::vector<std::string> object_ids = {"object_pickup", "object_avoid"};
	std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map = pick_and_place.planning_scene_interface.getObjects(object_ids);
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.resize(2);
	collision_objects[0] = collision_objects_map["object_pickup"];
		
	// Determine the pose of the goal, 
	grasping.determine_goal_pose(1, collision_objects);
	grasping.visualize();

	path_planning.set_goal(grasping.goal_pose);
	path_planning.plan();
	path_planning.execute();


	// double offset2 = 0.13;

	sleep(5);
	grasping.close_gripper();
	grasping.attach_object();

	
	ros::spinOnce();

	
	path_planning.execute();
	sleep(5);
	grasping.close_gripper();
	grasping.attach_object();


	// Code for placing objects	
	pick = 0;
	grasping.determine_goal_pose(0, collision_objects);
	grasping.visualize();

	path_planning.set_goal(grasping.goal_pose);
	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);
	path_planning.plan();
	
	path_planning.execute();

	// Sleep is necessary because in some motions the gripper will close when motion is still being executed
	sleep(5);
	grasping.open_gripper();
	
	
	return EXIT_SUCCESS;
}