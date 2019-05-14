#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>
#include <vector>
#include <map>


#ifndef AUTONOMOUS_PICK_PLACE_H
#define AUTONOMOUS_PICK_PLACE_H

class AutonomousPickPlace 
{
	public:
		std::string planning_group = "arm_torso";
		moveit::planning_interface::MoveGroupInterface move_group{planning_group};

		
		moveit_visual_tools::MoveItVisualTools visual_tools{move_group.getPlanningFrame(),"/rviz_visual_markers"};
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


		
		AutonomousPickPlace();
		~AutonomousPickPlace();

	private:
		
};

#endif