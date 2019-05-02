#include "path_planning.h"

PathPlanning::PathPlanning()
{
	ROS_INFO_STREAM("Path planning initialized");

	spinner.start();

}

PathPlanning::~PathPlanning()
{
	spinner.stop();
}

void PathPlanning::set_parameters(double planning_time, double vel_scale_factor, std::string reference_frame, std::string planner_id)
{
  	move_group.setPlanningTime(planning_time);
  	move_group.setPlannerId(planner_id);
  	move_group.setPoseReferenceFrame(reference_frame);	
  	move_group.setStartStateToCurrentState();
  	move_group.setMaxVelocityScalingFactor(vel_scale_factor);
}	


void PathPlanning::set_goal(geometry_msgs::PoseStamped goal_pose)
{
	move_group.setPoseTarget(goal_pose);
	ROS_INFO_STREAM("\t Target pose set to: " << move_group.getPoseTarget(move_group.getEndEffectorLink()));

}

void PathPlanning::plan()
{
	ROS_INFO_STREAM("Planning to move " <<
	              move_group.getEndEffectorLink() << " to a target pose expressed in " <<
	              move_group.getPlanningFrame());

	auto success = move_group.plan(my_plan);

	if (!success)
	{
		throw std::runtime_error("No plan found");
	}

	ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
}

void PathPlanning::execute()
{


	// Execute the plan
  	ros::Time start = ros::Time::now();

	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the "
	                    "robot");
  	move_group.move();

  	ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
}



void PathPlanning::add_objects(std::vector<moveit_msgs::CollisionObject> collision_objects)
{
	ROS_INFO_STREAM("\t"<<"CHECK");

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

