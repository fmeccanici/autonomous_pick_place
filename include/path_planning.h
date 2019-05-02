#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>

#include "autonomous_pick_place.h"


#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

class PathPlanning : public AutonomousPickPlace
{
	public:
		ros::NodeHandle nh;
		ros::AsyncSpinner spinner{1};

		bool visualization;

		geometry_msgs::PoseStamped goal_pose;
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
		PathPlanning();
		~PathPlanning();

		void set_parameters(double planning_time, double vel_scale_factor, 
			std::string reference_frame, std::string planner_id);
		
		void set_goal(geometry_msgs::PoseStamped goal_pose);
		void plan();
		void execute();
		void visualize();
		void add_objects(std::vector<moveit_msgs::CollisionObject> collision_objects);
};

#endif