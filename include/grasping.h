#include <ros/ros.h>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Int32.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "autonomous_pick_place.h"

#ifndef GRASPING_H
#define GRASPING_H

class Grasping : public AutonomousPickPlace
{
	public:
        ros::NodeHandle n;
        
        ros::Subscriber master_state_sub;
        ros::Publisher grip_goal_pub;
        moveit::planning_interface::MoveGroupInterface gripper_group{"gripper"};

        moveit_msgs::CollisionObject object_pickup;
        geometry_msgs::PoseStamped goal_pose;

        int rate, wb_count, current, en_torque, grip, en_grip, increment;
        double threshold_time;
        std_msgs::Int32 wb_m;

        dynamixel_workbench_msgs::DynamixelCommand en_grip_torque, grip_pos_goal, grip_current_goal;
        std::vector<moveit_msgs::Grasp> grasps;
        

        moveit_visual_tools::MoveItVisualTools visual_tools{move_group.getPlanningFrame(),"/rviz_visual_markers"};
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

	Grasping();
	~Grasping();
	void determine_goal_pose(bool pick_or_place, std::vector<moveit_msgs::CollisionObject> collision_objects); 
        void visualize();
        void open_gripper();
        void close_gripper();
        void attach_object();
        void detach_object();

        void open_gripper_sim(trajectory_msgs::JointTrajectory& posture);
        void close_gripper_sim(trajectory_msgs::JointTrajectory& posture);
        void pick();
        void place();
};

#endif