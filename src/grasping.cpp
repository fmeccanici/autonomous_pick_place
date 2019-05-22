#include "grasping.h"


Grasping::Grasping()
{}

Grasping::~Grasping()
{}

void Grasping::determine_goal_pose(bool pick_or_place, std::vector<moveit_msgs::CollisionObject> collision_objects)
{
	
	goal_pose.header.frame_id = move_group.getPlanningFrame();


	ROS_INFO_STREAM("\t" << collision_objects.size());
	
	for (unsigned int i = 0; i < collision_objects.size(); i++)
	{

		if (collision_objects[i].id == "object_pickup" && pick_or_place == 1)
		{
			object_pickup = collision_objects[i];

			/*
			// Random feasible pose
			goal_pose.pose.position.x = 0.43;
			goal_pose.pose.position.y = -0.08;
			goal_pose.pose.position.z = 0.33; //0.717439;
			goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, 0);
			*/

			// goal_pose.pose.orientation.x = -0.000238538;
			// goal_pose.pose.orientation.y = -0.01;
			// goal_pose.pose.orientation.z = 0.003;//-5.83127e-05;
			

			// Assume for now a simple grasping strategy always pick up from above: rotation of pi/2 around y axis
			
			
			goal_pose.pose.position.x = collision_objects[i].primitive_poses[0].position.x;
			goal_pose.pose.position.y = collision_objects[i].primitive_poses[0].position.y;
			goal_pose.pose.position.z = collision_objects[i].primitive_poses[0].position.z + collision_objects[i].primitives[0].dimensions[2]/2 + 0.13;

			/*
			goal_pose.pose.position.x = collision_objects[i].primitive_poses[0].position.x - 0.2;
			goal_pose.pose.position.y = collision_objects[i].primitive_poses[0].position.y;
			goal_pose.pose.position.z = collision_objects[i].primitive_poses[0].position.z + collision_objects[i].primitives[0].dimensions[2]/4;
			*/
			ROS_INFO_STREAM("\t" << goal_pose.pose.position.z);

			double roll = 0;
			double pitch = M_PI_2;
			// double pitch = 0;

			double yaw = 0;

			goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
			
			

		}
		else if (collision_objects[i].id == "object_avoid" && pick_or_place == 0)
		{

			goal_pose.pose.position.x = collision_objects[i].primitive_poses[0].position.x;
			goal_pose.pose.position.y = collision_objects[i].primitive_poses[0].position.y;
			goal_pose.pose.position.z = collision_objects[i].primitive_poses[0].position.z + collision_objects[i].primitives[0].dimensions[2]/2 + 0.4;

			ROS_INFO_STREAM("\t goal_pose, x: " << goal_pose.pose.position.x << "y: " << goal_pose.pose.position.y << "z: " << goal_pose.pose.position.z);
			double roll = 0;
			double pitch = M_PI_2;
			double yaw = 0;
			goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
			

		}
	}
}

void Grasping::attach_object()
{
	ROS_INFO_STREAM("\t" << "pickup_id: " << object_pickup.id);
	move_group.attachObject(object_pickup.id);
	visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
	visual_tools.trigger();

	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
	                    "robot");
}

void Grasping::detach_object()
{
	move_group.detachObject(object_pickup.id);
	visual_tools.publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
	visual_tools.trigger();

	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
	                    "robot");
}

void Grasping::visualize()
{
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();

	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

	visual_tools.trigger();

	ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	visual_tools.publishAxisLabeled(goal_pose.pose, "goal_pose");
	
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}

void Grasping::open_gripper()
{
	grip_goal_pub = n.advertise<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_single_manager/motor_command", 10);


	// init variables
	rate = 100;
	increment = 0;
	threshold_time = 0.3; // button push time [s] before incremental mode starts
	wb_m.data = 0;
	wb_count = 1;
	en_grip = 0;
	en_torque = 1;

	// grip determines the open or close
	// 0 is open 1600 is close
	grip = 0;
	// grip = 200;
	// grip = 900;

	current = 75;
	
	en_grip_torque.value = 0;
	grip_current_goal.value = 0;
	grip_pos_goal.value = 0;

	sleep(1);

	ros::Rate loop_rate(rate);


    // Gripper action
    {
        // Enable gripper torque
        en_grip_torque.addr_name = "torque_enable";
        en_grip_torque.value = en_torque;
        grip_goal_pub.publish(en_grip_torque);
    
        // Gripper goal current
        grip_current_goal.addr_name = "goal_current";
        grip_current_goal.value = current;
        grip_goal_pub.publish(grip_current_goal);
    
        // Gripper goal position
        grip_pos_goal.addr_name = "goal_position";
        grip_pos_goal.value = grip;
        grip_goal_pub.publish(grip_pos_goal);
    }

    loop_rate.sleep();
    ros::spinOnce();
}       		

void Grasping::close_gripper()
{
	grip_goal_pub = n.advertise<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_single_manager/motor_command", 10);

	// init variables
	rate = 100;
	increment = 0;
	threshold_time = 0.3; // button push time [s] before incremental mode starts
	wb_m.data = 0;
	wb_count = 1;
	en_grip = 0;
	en_torque = 1;

	// grip determines the open or close
	// 0 is open 1600 is close
	
	grip = 2000;
	// grip = 850;

	current = 75;
	
	en_grip_torque.value = 0;
	grip_current_goal.value = 0;
	grip_pos_goal.value = 0;

	sleep(1);

	ros::Rate loop_rate(rate);


    // Gripper action
    {
        // Enable gripper torque
        en_grip_torque.addr_name = "torque_enable";
        en_grip_torque.value = en_torque;
        grip_goal_pub.publish(en_grip_torque);
    
        // Gripper goal current
        grip_current_goal.addr_name = "goal_current";
        grip_current_goal.value = current;
        grip_goal_pub.publish(grip_current_goal);
    
        // Gripper goal position
        grip_pos_goal.addr_name = "goal_position";
        grip_pos_goal.value = grip;
        grip_goal_pub.publish(grip_pos_goal);
    }

    loop_rate.sleep();
    ros::spinOnce();
}   


void Grasping::open_gripper_sim(trajectory_msgs::JointTrajectory& posture)
{
	 // BEGIN_SUB_TUTORIAL open_gripper
	  /* Add both finger joints of panda robot. */
	  posture.joint_names.resize(2);
	  posture.joint_names[0] = "gripper_finger_tip_left_joint";
	  posture.joint_names[1] = "gripper_finger_tip_right_joint";

	  /* Set them as open, wide enough for the object to fit. */
	  posture.points.resize(1);
	  posture.points[0].positions.resize(2);
	  posture.points[0].positions[0] = 0.04;
	  posture.points[0].positions[1] = 0.04;
	  posture.points[0].time_from_start = ros::Duration(0.5);
	// END_SUB_TUTORIAL	
}

void Grasping::close_gripper_sim(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_finger_tip_left_joint";
  posture.joint_names[1] = "gripper_finger_tip_right_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
// END_SUB_TUTORIAL	
}

void Grasping::pick()
{
  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  // of the cube). |br|
  // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  // extra padding)
  	grasps.resize(1);
  	grasps[0].grasp_pose.header.frame_id = move_group.getPlanningFrame();
	double roll = 0;
	double pitch = M_PI_2;
	double yaw = 0;



	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
	grasps[0].grasp_pose.pose.position.x = 0.415;
	grasps[0].grasp_pose.pose.position.y = 0;
	grasps[0].grasp_pose.pose.position.z = 0.4;

	// Setting pre-grasp approach
	// ++++++++++++++++++++++++++
	/* Defined with respect to frame_id */
	grasps[0].pre_grasp_approach.direction.header.frame_id = move_group.getPlanningFrame();
	/* Direction is set as positive x axis */
	grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
	grasps[0].pre_grasp_approach.min_distance = 0.095;
	grasps[0].pre_grasp_approach.desired_distance = 0.115;

	// Setting post-grasp retreat
	// ++++++++++++++++++++++++++
	/* Defined with respect to frame_id */
	grasps[0].post_grasp_retreat.direction.header.frame_id = move_group.getPlanningFrame();
	/* Direction is set as positive z axis */
	grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
	grasps[0].post_grasp_retreat.min_distance = 0.1;
	grasps[0].post_grasp_retreat.desired_distance = 0.25;

	// Setting posture of eef before grasp
	// +++++++++++++++++++++++++++++++++++
	// open_gripper_sim(grasps[0].pre_grasp_posture);
	// END_SUB_TUTORIAL

	// BEGIN_SUB_TUTORIAL pick2
	// Setting posture of eef during grasp
	// +++++++++++++++++++++++++++++++++++
	// close_gripper_sim(grasps[0].grasp_posture);
	// END_SUB_TUTORIAL

	// BEGIN_SUB_TUTORIAL pick3
	// Set support surface as table1.
	// move_group.setSupportSurfaceName("table1");
	// Call pick to pick up the object using the grasps given
	move_group.pick("object_pickup", grasps);
	// END_SUB_TUTORIAL	

}

void Grasping::place()
{
	// BEGIN_SUB_TUTORIAL place
	// TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
	// location in
	// verbose mode." This is a known issue and we are working on fixing it. |br|
	// Create a vector of placings to be attempted, currently only creating single place location.
	std::vector<moveit_msgs::PlaceLocation> place_location;
	place_location.resize(1);

	// Setting place location pose
	// +++++++++++++++++++++++++++
	place_location[0].place_pose.header.frame_id = move_group.getPlanningFrame();
	tf2::Quaternion orientation;
	orientation.setRPY(0, M_PI_2, 0);
	place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

	/* While placing it is the exact location of the center of the object. */
	place_location[0].place_pose.pose.position.x = 0.415;
	place_location[0].place_pose.pose.position.y = 0;
	place_location[0].place_pose.pose.position.z = 0.4;


	// place_location[0].place_pose.pose.position.x = goal_pose.pose.position.x;
	// place_location[0].place_pose.pose.position.y = goal_pose.pose.position.y;
	// place_location[0].place_pose.pose.position.z = goal_pose.pose.position.z;



	// Setting pre-place approach
	// ++++++++++++++++++++++++++
	/* Defined with respect to frame_id */
	place_location[0].pre_place_approach.direction.header.frame_id = move_group.getPlanningFrame();
	/* Direction is set as negative z axis */
	place_location[0].pre_place_approach.direction.vector.z = -1.0;
	place_location[0].pre_place_approach.min_distance = 0.095;
	place_location[0].pre_place_approach.desired_distance = 0.115;

	// Setting post-grasp retreat
	// ++++++++++++++++++++++++++
	/* Defined with respect to frame_id */
	place_location[0].post_place_retreat.direction.header.frame_id = move_group.getPlanningFrame();
	/* Direction is set as negative y axis */
	place_location[0].post_place_retreat.direction.vector.y = -1.0;
	place_location[0].post_place_retreat.min_distance = 0.1;
	place_location[0].post_place_retreat.desired_distance = 0.25;




	// Setting posture of eef after placing object
	// +++++++++++++++++++++++++++++++++++++++++++
	/* Similar to the pick case */
	//open_gripper_sim(place_location[0].post_place_posture);

	// Set support surface as table2.
	// group.setSupportSurfaceName("table2");
	// Call place to place the object using the place locations given.
	move_group.place("object_pickup", place_location);
	// END_SUB_TUTORIAL	
}