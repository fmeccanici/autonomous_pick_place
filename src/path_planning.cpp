#include "path_planning.h"

PathPlanning::PathPlanning()
{
	ROS_INFO_STREAM("Path planning initialized");

	int argc = 0;
	char** argv = {};

	ros::init(argc, argv, "path_planning_node");

	spinner.start();

}


PathPlanning::~PathPlanning()
{
	spinner.stop();
}


void PathPlanning::start_spinner()
{
	
}


void PathPlanning::stop_spinner()
{
	
}



void PathPlanning::set_parameters(double planning_time, double vel_scale_factor, std::string reference_frame, std::string planner_id)
{
  	move_group.setPlanningTime(planning_time);
  	move_group.setPlannerId(planner_id);
  	move_group.setPoseReferenceFrame(reference_frame);	
  	move_group.setStartStateToCurrentState();
  	move_group.setMaxVelocityScalingFactor(vel_scale_factor);
	benchmarking_plannerid_publisher.publish(move_group.getPlannerId());
  	myfile.open("/home/fmeccanici/Documents/autonomous_manipulation/rosbags/motion_planners_evaluation/planner_id.txt");
  	myfile << move_group.getPlannerId();
  	myfile.close();

  	// benchmark_motion_planner_publisher.publish(planner_id);
  	// ros::spinOnce();
  	// ROS_INFO_STREAM("\t SPINNED");

}	


void PathPlanning::set_goal(geometry_msgs::PoseStamped goal_pose)
{
	move_group.setPoseTarget(goal_pose);
	ROS_INFO_STREAM("\t Target pose set to: " << move_group.getPoseTarget(move_group.getEndEffectorLink()));
}

void PathPlanning::plan()
{
	// start_spinner();

	ROS_INFO_STREAM("Planning to move " <<
	              move_group.getEndEffectorLink() << " to a target pose expressed in " <<
	              move_group.getPlanningFrame());

	auto success = move_group.plan(my_plan);

	if (!success)
	{
		throw std::runtime_error("No plan found");
	}

	ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

	// benchmark_msg.planning_time = my_plan.planning_time_;

	benchmarking_time_publisher.publish(my_plan.planning_time_);
	ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points.size());

	for (unsigned i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++)
	{
		// ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points.size());

		// ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points[i]);
		benchmarking_trajectory_publisher.publish(my_plan.trajectory_.joint_trajectory.points[i]);
	}
	
	benchmarking_joint_names_publisher.publish(my_plan.trajectory_.joint_trajectory.joint_names);

	// ros::spinOnce();
	// sleep(10);
	// stop_spinner();
	

}


void PathPlanning::execute()
{

	// Execute the plan
  	ros::Time start = ros::Time::now();

	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the "
	                    "robot");
  	move_group.move();

  	double execution_time = (ros::Time::now() - start).toSec();

  	benchmarking_execution_time_publisher.publish(execution_time);

  	myfile.open("/home/fmeccanici/Documents/autonomous_manipulation/rosbags/motion_planners_evaluation/execution_time.txt");
  	myfile << execution_time;
  	myfile.close();

  	ROS_INFO_STREAM("Motion duration: " << execution_time);

  	ros::spinOnce();
}

void PathPlanning::add_floor()
{
	
	floor.id = "floor";
	floor.header.frame_id = move_group.getPlanningFrame();

	floor.primitives.resize(1);
	floor.primitives[0].type = floor.primitives[0].BOX;
	floor.primitives[0].dimensions.resize(3); 	

	floor.primitives[0].dimensions[0] = 5;
	floor.primitives[0].dimensions[1] = 5;
	floor.primitives[0].dimensions[2] = 0; //0.05;

	floor.primitive_poses.resize(1);
	floor.primitive_poses[0].position.x = 0;
	floor.primitive_poses[0].position.y = 0;
	floor.primitive_poses[0].position.z = 0;	

}
void PathPlanning::add_objects(std::vector<moveit_msgs::CollisionObject> collision_objects)
{

	add_floor();

	collision_objects.push_back(floor);
	ROS_INFO_NAMED("tutorial", "Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);

	// // Pick pipeline
	// std::vector<moveit_msgs::Grasp> grasps;
	// grasps.resize(1);

	

	// // Setting grasp pose
	// grasps[0].grasp_pose.header.frame_id = move_group.getPlanningFrame();	
	// tf2::Quaternion orientation;
	// orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
	
	// // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
	// grasps[0].grasp_pose.pose.position.x = 0.415;
	// grasps[0].grasp_pose.pose.position.y = 0;
	// grasps[0].grasp_pose.pose.position.z = 0.5;

	// // Set pre-grasp approach

	// /* Defined with respect to frame_id */
	// grasps[0].pre_grasp_approach.direction.header.frame_id = move_group.getPlanningFrame();
	// /* Direction is set as positive x axis */
	// grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
	// grasps[0].pre_grasp_approach.min_distance = 0.095;
	// grasps[0].pre_grasp_approach.desired_distance = 0.115;

	// //Setting post-grasp retreat
	// /* Defined with respect to frame_id */
	// grasps[0].post_grasp_retreat.direction.header.frame_id = move_group.getPlanningFrame();
	// /* Direction is set as positive z axis */
	// grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
	// grasps[0].post_grasp_retreat.min_distance = 0.1;
	// grasps[0].post_grasp_retreat.desired_distance = 0.25;


	// move_group.setSupportSurfaceName("table1");
	// Setting posture of eef before grasp
	// openGripper(grasps[0].pre_grasp_posture);

	// move_group.pick("object", grasps);
}

void PathPlanning::disable_collisions()
{
	collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
	// robot_state::RobotState copied_state = planning_scene.getCurrentState();
 //  	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


	// ros.wait_for_service("/get_planning_scene", 10.0);
	// ros.ServiceProxy("/get_planning_scene", planning_scene::GetPlanningScene);
	// request = planning_scene::PlanningSceneComponents(components=planning_scene::PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)

	// response = planning_scene::get_planning_scene(request)
	// acm = response.scene.allowed_collision_matrix

	collision_detection::AllowedCollision::Type allowed;


	ROS_INFO_STREAM("\t" << "allowed: " << acm.getAllowedCollision("gripper_finger_tip_left_joint", "object_pickup", allowed));

	
	acm.setEntry("gripper_finger_tip_left_joint", "object_pickup", true);
	acm.setEntry("gripper_finger_tip_right_joint", "object_pickup", true);
	acm.setEntry("gripper_finger_inner_right_joint", "object_pickup", true);
	acm.setEntry("gripper_finger_inner_left_joint", "object_pickup", true);
	acm.setEntry("gripper_finger_outer_left_joint", "object_pickup", true);
	acm.setEntry("gripper_finger_outer_right_joint", "object_pickup", true);
	acm.setEntry("gripper_joint", "object_pickup", true);
	acm.setEntry("gripper_body_link", "object_pickup", true);
	acm.setEntry("gripper_wrist_link", "object_pickup", true);
	acm.setEntry("wrist_ft_tool_link", "object_pickup", true);
	acm.setEntry("wrist_ft_link", "object_pickup", true);
	acm.setEntry("end_effector_cam_link", "object_pickup", true);
	acm.setEntry("end_effector_cam_optical_frame", "object_pickup", true);
	acm.setEntry("gripper_grasp_link", "object_pickup", true);
	acm.setEntry("gripper_motor_link", "object_pickup", true);
	acm.setEntry("arm_tool_link", "object_pickup", true);
	acm.setEntry("gripper_finger_tip_left_link", "object_pickup", true);
	acm.setEntry("gripper_finger_inner_right_link", "object_pickup", true);
	acm.setEntry("gripper_finger_tip_right_link", "object_pickup", true);

	std::vector<std::string> default_entry_names;
	// std::vector<moveit_msgs::AllowedCollisionEntry> entry_values;
	std::vector<bool> default_entry_values;

	int size = 4;

	default_entry_names.resize(size);
	default_entry_values.resize(size);

 	default_entry_names[0] = "gripper_finger_tip_right_link";
 	default_entry_names[1] = "gripper_link";
	default_entry_names[2] = "gripper_wrist_link";
	default_entry_names[3] = "gripper_finger_tip_right_link";

 	default_entry_values[0] = true;
 	default_entry_values[1] = true;
 	default_entry_values[2] = true;
 	default_entry_values[3] = true;

 	// moveit_msgs::AllowedCollisionEntry entry;

 	// std::vector<bool> enabled;
 	// enabled.resize(1);
 	// enabled[0] = true;
 	// entry.enabled = enabled;
 	// entry_values[0] = entry;

 	planning_scene_msgs.allowed_collision_matrix.default_entry_names.resize(size);
 	planning_scene_msgs.allowed_collision_matrix.default_entry_values.resize(size);


	planning_scene_msgs.allowed_collision_matrix.default_entry_names[0] = default_entry_names[0];
	planning_scene_msgs.allowed_collision_matrix.default_entry_values[0] = default_entry_values[0];
	planning_scene_msgs.allowed_collision_matrix.default_entry_names[1] = default_entry_names[1];
	planning_scene_msgs.allowed_collision_matrix.default_entry_values[1] = default_entry_values[1];

	planning_scene_msgs.allowed_collision_matrix.default_entry_names[2] = default_entry_names[2];
	planning_scene_msgs.allowed_collision_matrix.default_entry_values[2] = default_entry_values[2];

	planning_scene_msgs.allowed_collision_matrix.default_entry_names[3] = default_entry_names[3];
	planning_scene_msgs.allowed_collision_matrix.default_entry_values[3] = default_entry_values[3];
	
	planning_scene_msgs.is_diff = true;

	ROS_INFO_STREAM("\t CHECK");
	planning_scene_diff_publisher.publish(planning_scene_msgs);

	// ROS_INFO_STREAM("\t" << acm.hasEntry("gripper_finger_outer_right_joint", "object_pickup"));
	ROS_INFO_STREAM("\t" << "allowed: " << acm.getAllowedCollision("gripper_finger_tip_left_joint", "object_pickup", allowed));



}

void PathPlanning::home()
{
	// joint space home position
	const robot_state::JointModelGroup* joint_model_group =
	    move_group.getCurrentState()->getJointModelGroup("arm_torso");

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();	
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// get current joint position
	/*
	const std::vector< std::string > joint_group_names = move_group.getJointNames();

	for (unsigned int i = 0; i < joint_group_positions.size(); i++)
	{
		ROS_INFO_STREAM("\t" << joint_group_positions[i]);
		ROS_INFO_STREAM("\t" << joint_group_names[i]);

		
	}
	*/

	// [torso_lift_joint, arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint, arm_7_joint]
	joint_group_positions = {0.115151, 2.36694, 0.37063, -0.0277066, 1.92753, 0.537318, -0.0297211, 0.270001};
	//joint_group_positions = {2.366953473745571, 0.37061498105508595, -0.027813971682339046, 1.9275275258384308, 0.5373180664365675, -0.02972114173012655, 0.2700012783286766, 0.005061454824999999, -0.8300785912999998, 0.11505448934323464, 0.03042328042328042, 0.08597883597883597, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

	move_group.setJointValueTarget(joint_group_positions);

	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	// cartesian home position
	/*
	geometry_msgs::PoseStamped home_pose;

	home_pose.header.frame_id = move_group.getPlanningFrame();
	home_pose.pose.position.x = 0.374520550483;
	home_pose.pose.position.y = 0.377399909297;
	home_pose.pose.position.z = 0.48811762453;
	home_pose.pose.orientation.x = 0.000370039741603;
	home_pose.pose.orientation.y = 0.707193378361;
	home_pose.pose.orientation.z = -0.000147597484232;
	home_pose.pose.orientation.w = 0.707020061164;
	move_group.setPoseTarget(home_pose);

	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();

	ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	visual_tools.publishAxisLabeled(goal_pose.pose, "goal_pose");
	
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	ROS_INFO_STREAM("\t Home pose set to: " << move_group.getPoseTarget(move_group.getEndEffectorLink()));
*/
	execute();
}

