#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <aruco_msgs/MarkerArray.h>


#include "autonomous_pick_place.h"


#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

class ObjectDetection : public AutonomousPickPlace
{
	public:
		geometry_msgs::PoseStamped goal_pose;
		std::vector<moveit_msgs::CollisionObject> collision_objects;

		ObjectDetection();
		~ObjectDetection();

		void detect_objects();

	private:
		void subscriber_callback(const aruco_msgs::MarkerArray &mk_array);

};

#endif