#ifndef BENCHMARKING_H
#define BENCHMARKING_H

#include <exception>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "autonomous_pick_place/benchmarking.h"

#include <string>

#include "autonomous_pick_place.h"


class Benchmarking
{

	public:
		// Autonomous pick and place
		ros::NodeHandle nh;
		ros::AsyncSpinner spinner{1};

		moveit::planning_interface::MoveGroupInterface move_group;
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		
		moveit_visual_tools::MoveItVisualTools visual_tools{move_group.getPlanningFrame(),"/rviz_visual_markers"};
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

		// ros::Publisher benchmark_motion_planner_publisher = nh.advertise<std_msgs::String>("motion_pla


		ros::Publisher benchmarking_message_publisher = nh.advertise<autonomous_pick_place::benchmarking>("motion_planning_benchmarking",1);
		ros::Subscriber joint_state_subscriber = nh.subscribe("/joint_states", 1000, &Benchmarking::subscriber_callback, this);



		Benchmarking();
		~Benchmarking();

		autonomous_pick_place::benchmarking set_parameters();
		void start();
		void subscriber_callback(const sensor_msgs::JointState &joint_states);
			

};

#endif