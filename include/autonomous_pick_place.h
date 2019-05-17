#ifndef AUTONOMOUS_PICK_PLACE_H
#define AUTONOMOUS_PICK_PLACE_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/String.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>
#include <vector>
#include <map>


class AutonomousPickPlace 
{
	public:
		ros::NodeHandle nh;
		ros::AsyncSpinner spinner{1};

		std::string planning_group = "arm_torso" ;
		moveit::planning_interface::MoveGroupInterface move_group{planning_group};
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		
		moveit_visual_tools::MoveItVisualTools visual_tools{move_group.getPlanningFrame(),"/rviz_visual_markers"};
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

		// ros::Publisher benchmark_motion_planner_publisher = nh.advertise<std_msgs::String>("motion_planning_benchmark/motion_planner",1);

		AutonomousPickPlace();
		~AutonomousPickPlace();

	private:
		
};

#endif