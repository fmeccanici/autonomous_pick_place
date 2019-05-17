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
	// bool benchmark = 0;

	// if (benchmark == 1)
	// {
	// 	Benchmarking benchmarking;
	// 	benchmarking.start();
	// }



	ros::init(argc, argv, "autonomous_pick_place_node");
	

	// ros::NodeHandle nh;
	// ros::Publisher benchmarking_time_publisher = nh.advertise<std_msgs::Float64>("motion_planning_benchmarking/planning_time",1);
	// ros::Publisher benchmarking_trajectory_publisher = nh.advertise<moveit_msgs::RobotTrajectory>("motion_planning_benchmarking/trajectory",1);
	ros::NodeHandle nh;
	ros::Publisher benchmarking_plannerid_publisher = nh.advertise<std_msgs::String>("motion_planning_benchmarking/planner_id",1);

	ofstream myfile;


	// If benchmark parameter is set to true
	// ROS_INFO_STREAM(typeid(*argv[1]).name());
	// ROS_INFO_STREAM(argv[1]);


	// Otherwise just start the normal pipeline
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



	

	// publish planner id to benchmark topic
	// pick_and_place.benchmark_motion_planner_publisher.publish(planner_id);

	// ros::spinOnce();


	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);


	// if (argv[1] == "1")
	// if (benchmark==1)
	// {
	// 	ROS_INFO_STREAM("\t CHECK");
	// 	Benchmarking benchmark;
	// 	benchmark.start();
	// }


	// grasping.detach_object();
	grasping.open_gripper();

	path_planning.home();

	grasping.detach_object();

	object_detection.detect_objects();
	path_planning.add_objects(object_detection.collision_objects);

	std::vector<std::string> object_ids = {"object_pickup", "object_avoid"};
	std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map = pick_and_place.planning_scene_interface.getObjects(object_ids);

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.resize(2);

	collision_objects[0] = collision_objects_map["object_pickup"];
		
	
	grasping.determine_goal_pose(1, collision_objects);
	grasping.visualize();

	path_planning.set_goal(grasping.goal_pose);


	path_planning.plan();
	// benchmarking_time_publisher.publish(path_planning.my_plan.trajectory_);
	// benchmarking_trajectory_publisher.publish(path_planning.my_plan.planning_time_);

	myfile.open("/home/fmeccanici/moveit_ws/src/robot_models/marco/autonomous_pick_place/files/trajectory.txt");
	myfile << path_planning.my_plan.trajectory_ << "\n";
	myfile.close();

	myfile.open("/home/fmeccanici/moveit_ws/src/robot_models/marco/autonomous_pick_place/files/planning_time.txt");
	myfile << path_planning.my_plan.planning_time_ << "\n";
	myfile.close();


	myfile.open("/home/fmeccanici/moveit_ws/src/robot_models/marco/autonomous_pick_place/files/planner_id.txt");
	myfile << planner_id << "\n";
	myfile.close();
	benchmarking_plannerid_publisher.publish(planner_id);
	ros::spinOnce();

	path_planning.execute();
	sleep(5);
	grasping.close_gripper();
	grasping.attach_object();




	// std::vector<std::string> object_ids = {"object_pickup", "object_avoid"};
	// std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map = pick_and_place.planning_scene_interface.getObjects(object_ids);

	// std::vector<moveit_msgs::CollisionObject> collision_objects;
	// collision_objects.resize(2);

	// collision_objects[0] = collision_objects_map["object_pickup"];
	// collision_objects[1] = collision_objects_map["object_avoid"];

	
	//grasping.determine_goal_pose(1, object_detection.collision_objects);
	grasping.determine_goal_pose(0, collision_objects);

	grasping.visualize();
	//grasping.attach_object();
	path_planning.disable_collisions();


	// grasping.open_gripper();
	path_planning.set_goal(grasping.goal_pose);
	path_planning.set_parameters(planning_time, vel_scale_factor, reference_frame, planner_id);
	path_planning.plan();

	
	
	path_planning.execute();
	sleep(5);
	grasping.open_gripper();
	

	
	
	return EXIT_SUCCESS;
}