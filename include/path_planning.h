#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


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
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		robot_model_loader::RobotModelLoader robot_model_loader{"robot_description"};
		robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
		planning_scene::PlanningScene planning_scene{kinematic_model};

		moveit_msgs::PlanningScene planning_scene_msgs;
		moveit_msgs::CollisionObject floor;
		
		ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
		
		PathPlanning();
		~PathPlanning();

		void set_parameters(double planning_time, double vel_scale_factor, 
			std::string reference_frame, std::string planner_id);
		
		void set_goal(geometry_msgs::PoseStamped goal_pose);
		void plan();
		void execute();
		void visualize();
		void add_objects(std::vector<moveit_msgs::CollisionObject> collision_objects);
		void disable_collisions();
		void home();
		void add_floor();
};

#endif