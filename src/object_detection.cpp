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

	// make sure to also have collision objects to work with when aruco markers are not detected 
	// you know this from previous detection and the attached object
	// if (mk_array.markers.size() == 0)
	// {
	// 	collision_objects.resize(2);

	// 	// object pickup
	// 	collision_objects[0].id = "object_pickup";
	// 	collision_objects[0].header.frame_id = move_group.getPlanningFrame();

	// 	// Define the primitive and its dimensions. 
	// 	collision_objects[0].primitives.resize(1);
	// 	collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	// 	collision_objects[0].primitives[0].dimensions.resize(3);

	// 	// new object that doesnt use gripper to be picked up
	// 	collision_objects[0].primitives[0].dimensions[0] = 0.15;
	// 	collision_objects[0].primitives[0].dimensions[1] = 0.12;
	// 	collision_objects[0].primitives[0].dimensions[2] = 0.24; //0.05;	

	// 	// Define the pose of the object. 
	// 	collision_objects[1].primitive_poses.resize(1);
	// 	std::vector<std::string> ids = {"object_pickup", "object_avoid"};

	// 	collision_objects[1].primitive_poses[0].position.x = planning_scene_interface.getObjectPoses(ids)[0].position;


	// 	// object avoid
	// 	collision_objects[1].id = "object_avoid";
	// 	collision_objects[1].header.frame_id = move_group.getPlanningFrame();

	// 	// Define the primitive and its dimensions. 
	// 	collision_objects[1].primitives.resize(1);
	// 	collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
	// 	collision_objects[1].primitives[0].dimensions.resize(3);


	// 	collision_objects[1].primitives[0].dimensions[0] = 0.30;
	// 	collision_objects[1].primitives[0].dimensions[1] = 0.23; 
	// 	collision_objects[1].primitives[0].dimensions[2] = 0.40; 

	// 	// Define the pose of the object. 
	// 	collision_objects[1].primitive_poses.resize(1);


	// 	collision_objects[1].primitive_poses[0] = planning_scene_interface.getObjectPoses(ids)[1].position;

	// } 
	for (unsigned int i = 0; i <= mk_array.markers.size(); i++)
	{
		int j = i+1;

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

			// with margin
			collision_objects[i].primitives[0].dimensions[0] = 0.1;
			collision_objects[i].primitives[0].dimensions[1] = 0.1;
			collision_objects[i].primitives[0].dimensions[2] = 0.4; 
		

			// new object that doesnt use gripper to be picked up
			// collision_objects[i].primitives[0].dimensions[0] = 0.12;
			// collision_objects[i].primitives[0].dimensions[1] = 0.15;
			// collision_objects[i].primitives[0].dimensions[2] = 0.24; 		

			// Without margin
			// collision_objects[i].primitives[0].dimensions[0] = 0.075;
			// collision_objects[i].primitives[0].dimensions[1] = 0.075;
			// collision_objects[i].primitives[0].dimensions[2] = 0.320; 

			// Define the pose of the object. 
			collision_objects[i].primitive_poses.resize(1);
			collision_objects[i].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x;
			collision_objects[i].primitive_poses[0].position.y = mk_array.markers[i].pose.pose.position.y;
			collision_objects[i].primitive_poses[0].position.z = mk_array.markers[i].pose.pose.position.z - collision_objects[i].primitives[0].dimensions[2]/2; //0.025;


		}

		if (mk_array.markers[i].id == 3)
		{

			collision_objects[j].id = "object_avoid";
			collision_objects[j].header.frame_id = move_group.getPlanningFrame();

			// Define the primitive and its dimensions. 
			collision_objects[j].primitives.resize(1);
			collision_objects[j].primitives[0].type = collision_objects[j].primitives[0].BOX;
			collision_objects[j].primitives[0].dimensions.resize(3);			

			// with margin 
			collision_objects[j].primitives[0].dimensions[0] = 0.30;
			collision_objects[j].primitives[0].dimensions[1] = 0.28; 
			collision_objects[j].primitives[0].dimensions[2] = 0.45; 

			// without margin
			// collision_objects[j].primitives[0].dimensions[0] = 0.30;
			// collision_objects[j].primitives[0].dimensions[1] = 0.23; 
			// collision_objects[j].primitives[0].dimensions[2] = 0.40; 


			// Define the pose of the object. 
			collision_objects[j].primitive_poses.resize(1);
			collision_objects[j].primitive_poses[0].position.x = mk_array.markers[i].pose.pose.position.x;
			collision_objects[j].primitive_poses[0].position.y = mk_array.markers[i].pose.pose.position.y;
			collision_objects[j].primitive_poses[0].position.z = mk_array.markers[i].pose.pose.position.z - collision_objects[j].primitives[0].dimensions[2]/2;// + 0.115; //0.025;
		

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