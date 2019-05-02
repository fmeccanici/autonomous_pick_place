#include "object_detection.h"

ObjectDetection::ObjectDetection()
{}

ObjectDetection::~ObjectDetection()
{}

// Simple 


void ObjectDetection::subscriber_callback(const aruco_msgs::MarkerArray &mk_array)
{

	collision_objects.resize(mk_array.markers.size()+1);
	// collision_objects.resize(mk_array.markers.size()+1);
	ROS_INFO_STREAM("\t" << mk_array.markers.size()+1);

	for (unsigned int i = 0; i <= mk_array.markers.size(); i++)
	{
		ROS_INFO_STREAM("\t" << i);
		if (mk_array.markers[i].id == 0)
		{
			collision_objects[i].id = "object_pickup";
			collision_objects[i].header.frame_id = move_group.getPlanningFrame();

			// Define the primitive and its dimensions. 
			// cup
			/*
			collision_objects[i].primitives.resize(1);
			collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
			collision_objects[i].primitives[0].dimensions.resize(3);
			collision_objects[i].primitives[0].dimensions[0] = 0.081;
			collision_objects[i].primitives[0].dimensions[1] = 0.081;
			collision_objects[i].primitives[0].dimensions[2] = 0.095; //0.05;

			// Define the pose of the object. 
			collision_objects[i].primitive_poses.resize(1);
			collision_objects[i].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x - 0.0625 - 0.0927 - 0.2;
			collision_objects[i].primitive_poses[0].position.y = mk_array.markers[i].pose.pose.position.y;
			collision_objects[i].primitive_poses[0].position.z = mk_array.markers[i].pose.pose.position.z + 0.0475; //0.025;
		*/
			// long box
			collision_objects[i].primitives.resize(1);
			collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
			collision_objects[i].primitives[0].dimensions.resize(3);
			collision_objects[i].primitives[0].dimensions[0] = 0.075;
			collision_objects[i].primitives[0].dimensions[1] = 0.075;
			collision_objects[i].primitives[0].dimensions[2] = 0.320; //0.05;

			// Define the pose of the object. 
			collision_objects[i].primitive_poses.resize(1);
			// collision_objects[i].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x + 0.0625 + 0.0927 - 0.2 + 0.05;
			collision_objects[i].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x + 0.0625 + 0.0927;
			collision_objects[i].primitive_poses[0].position.y = mk_array.markers[i].pose.pose.position.y + 0.03;
			collision_objects[i].primitive_poses[0].position.z = mk_array.markers[i].pose.pose.position.z + 0.160; //0.025;

		}

		if (mk_array.markers[i].id == 0)
		{
			collision_objects[i+1].id = "object_avoid";
			collision_objects[i+1].header.frame_id = move_group.getPlanningFrame();

			// Define the primitive and its dimensions. 
			collision_objects[i+1].primitives.resize(1);
			collision_objects[i+1].primitives[0].type = collision_objects[i].primitives[0].BOX;
			collision_objects[i+1].primitives[0].dimensions.resize(3);
			// collision_objects[i].primitives[0].dimensions[0] = 0.40;
			// collision_objects[i].primitives[0].dimensions[1] = 0.30;
			// collision_objects[i].primitives[0].dimensions[2] = 0.23; //0.05;

			


			collision_objects[i+1].primitives[0].dimensions[0] = 0.30;
			collision_objects[i+1].primitives[0].dimensions[1] = 0.23; 
			collision_objects[i+1].primitives[0].dimensions[2] = 0.40 + 0.44;  //0.05;



			// collision_objects[i+1].primitives[0].dimensions[0] = 0.30;
			// collision_objects[i+1].primitives[0].dimensions[1] = 0.23; 
			// collision_objects[i+1].primitives[0].dimensions[2] = 0.40;  //0.05;

			// Define the pose of the object. 
			collision_objects[i+1].primitive_poses.resize(1);
			collision_objects[i+1].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x;
			collision_objects[i+1].primitive_poses[0].position.y = mk_array.markers[i].pose.pose.position.y;
			collision_objects[i+1].primitive_poses[0].position.z = mk_array.markers[i].pose.pose.position.z - collision_objects[i+1].primitives[0].dimensions[2]/2;// + 0.115; //0.025;
		
			// Define the pose of the object. 
			// collision_objects[i+1].primitive_poses.resize(1);
			// collision_objects[i+1].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x + collision_objects[i].primitives[0].dimensions[0]/2;
			// collision_objects[i+1].primitive_poses[0].position.y = mk_array.markers[i].pose.pose.position.y;
			// collision_objects[i+1].primitive_poses[0].position.z = mk_array.markers[i].pose.pose.position.z - (collision_objects[i].primitives[0].dimensions[2]/2 + collision_objects[i+1].primitives[0].dimensions[2]/2);// + 0.115; //0.025;

		}
	}
	sleep(2);
}


// void ObjectDetection::detect_objects()
// {
// 	ros::init(0, [], "object_detection_node");
// 	ros::NodeHandle n;

// 	ros::Subscriber sub = n.subscribe("/marker_detector/markers", 1000, subcriber_callback);

// 	collision_objects.resize(2);

// 	collision_objects[0].header.frame_id = move_group.getPlanningFrame();
// 	collision_objects[0].id = "object_avoid";
	
// 	collision_objects[0].primitives.resize(1);
// 	collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
// 	collision_objects[0].primitives[0].dimensions.resize(3);	
	
// 	collision_objects[0].primitives[0].dimensions[0] = 0.9;
// 	collision_objects[0].primitives[0].dimensions[1] = 1;
// 	collision_objects[0].primitives[0].dimensions[2] = 0.05;

// 	collision_objects[0].primitive_poses.resize(1);
// 	collision_objects[0].primitive_poses[0].position.x = 0.9;
// 	collision_objects[0].primitive_poses[0].position.y = 0;
// 	collision_objects[0].primitive_poses[0].position.z = 0.8;

// /*
// 	collision_objects[1].id = "object_pickup";
// 	collision_objects[1].header.frame_id = move_group.getPlanningFrame();

// 	// Define the primitive and its dimensions. 
// 	collision_objects[1].primitives.resize(1);
// 	collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
// 	collision_objects[1].primitives[0].dimensions.resize(3);
// 	collision_objects[1].primitives[0].dimensions[0] = 0.05;
// 	collision_objects[1].primitives[0].dimensions[1] = 0.05;
// 	collision_objects[1].primitives[0].dimensions[2] = 0.35; //0.05;

// 	// Define the pose of the object. 
// 	collision_objects[1].primitive_poses.resize(1);
// 	collision_objects[1].primitive_poses[0].position.x = 0.4;
// 	collision_objects[1].primitive_poses[0].position.y = 0;
// 	collision_objects[1].primitive_poses[0].position.z = 0.175; //0.025;
// 	*/


// }


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