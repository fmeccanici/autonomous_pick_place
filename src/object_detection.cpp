#include "object_detection.h"

ObjectDetection::ObjectDetection()
{}

ObjectDetection::~ObjectDetection()
{}

void ObjectDetection::subscriber_callback(const aruco_msgs::MarkerArray &mk_array)
{
	for (unsigned int i = 0; i <= mk_array.markers.size(); i++)
	{
		int j = i+1;

		ROS_INFO_STREAM("\t" << i);
		if (mk_array.markers[i].id == 0)
		{
			collision_objects[i].id = "object_pickup";
			collision_objects[i].header.frame_id = move_group.getPlanningFrame();
			
			// bolletje box
			collision_objects[i].primitives.resize(1);
			collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
			collision_objects[i].primitives[0].dimensions.resize(3);

			// without margin
			// collision_objects[i].primitives[0].dimensions[0] = 0.135;
			// collision_objects[i].primitives[0].dimensions[1] = 0.055;
			// collision_objects[i].primitives[0].dimensions[2] = 0.14; 

			// with margin
			collision_objects[i].primitives[0].dimensions[0] = 0.15;
			collision_objects[i].primitives[0].dimensions[1] = 0.09;
			collision_objects[i].primitives[0].dimensions[2] = 0.2; 

			// Define the pose of the object. 
			collision_objects[i].primitive_poses.resize(1);
			collision_objects[i].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x;
			collision_objects[i].primitive_poses[0].position.y = mk_array.markers[i].pose.pose.position.y;
			collision_objects[i].primitive_poses[0].position.z = mk_array.markers[i].pose.pose.position.z - collision_objects[i].primitives[0].dimensions[2]/2; 

		}

		if (mk_array.markers[i].id == 3)
		{

			collision_objects[j].id = "object_avoid";
			collision_objects[j].header.frame_id = move_group.getPlanningFrame();

			// Define the primitive and its dimensions. 
			collision_objects[j].primitives.resize(1);
			collision_objects[j].primitives[0].type = collision_objects[j].primitives[0].BOX;
			collision_objects[j].primitives[0].dimensions.resize(3);			

			// large box on top of two small boxes
			// with margin 
			collision_objects[j].primitives[0].dimensions[0] = 0.35;
			collision_objects[j].primitives[0].dimensions[1] = 0.28; 
			collision_objects[j].primitives[0].dimensions[2] = 0.65; 

			// without margin
			// collision_objects[j].primitives[0].dimensions[0] = 0.30;
			// collision_objects[j].primitives[0].dimensions[1] = 0.23; 
			// collision_objects[j].primitives[0].dimensions[2] = 0.40; 


			// Define the pose of the object. 
			collision_objects[j].primitive_poses.resize(1);
			collision_objects[j].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x;
			collision_objects[j].primitive_poses[0].position.y = mk_array.markers[i].pose.pose.position.y;
			collision_objects[j].primitive_poses[0].position.z = mk_array.markers[i].pose.pose.position.z - collision_objects[j].primitives[0].dimensions[2]/2;

		}
	}
	sleep(2);

}


void ObjectDetection::detect_objects()
{
		char** argv = {};
		int argc = 0;

		ros::init(argc, argv, "object_detection_node");
		ros::NodeHandle n;

		ros::Subscriber sub = n.subscribe("/marker_detector/markers", 1000, &ObjectDetection::subscriber_callback, this);

		ros::spinOnce();
		sleep(1);

}