#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_msgs/Float64MultiArray.h> 
#include <string>
#include <iostream>
#include <fstream>

#include "autonomous_pick_place.h"


class PathPlanning : public AutonomousPickPlace
{
	public:
		bool visualization;

		geometry_msgs::PoseStamped goal_pose;
		

		robot_model_loader::RobotModelLoader robot_model_loader{"robot_description"};
		robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
		planning_scene::PlanningScene planning_scene{kinematic_model};

		moveit_msgs::PlanningScene planning_scene_msgs;
		moveit_msgs::CollisionObject floor;
		
		// autonomous_pick_place::benchmark benchmark_msg;

		ros::Publisher benchmarking_time_publisher = nh.advertise<std_msgs::Float64>("motion_planning_benchmarking/planning_time",1);
		ros::Publisher benchmarking_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("motion_planning_benchmarking/trajectory",1);
		ros::Publisher benchmarking_joint_names_publisher = nh.advertise<std_msgs::String>("motion_planning_benchmarking/joint_names",1);
		ros::Publisher benchmarking_execution_time_publisher = nh.advertise<std_msgs::String>("motion_planning_benchmarking/joint_names",1);
  		ros::Publisher benchmarking_plannerid_publisher = nh.advertise<std_msgs::String>("motion_planning_benchmarking/planner_id",1);

		ros::Publisher planning_scene_diff_publisher = nh.advertise<std_msgs::Float64>("motion_planning_benchmarking/execution_time", 1);
		std::ofstream myfile;

		PathPlanning();
		~PathPlanning();

		void set_parameters(double planning_time, double vel_scale_factor, 
			std::string reference_frame, std::string planner_id);
		
		void set_goal(geometry_msgs::PoseStamped goal_pose);
		void plan();
		void planAndPublish();
		void execute();
		void visualize();
		void add_objects(std::vector<moveit_msgs::CollisionObject> collision_objects);
		void disable_collisions();
		void home();
		void add_floor();
		void start_spinner();
		void stop_spinner();
};

#endif