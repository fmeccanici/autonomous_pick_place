#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Log.h>




void callback(const rosgraph_msgs::Log log)
{
	// if (log.msg.find("Fail to write") == std::string::npos)
	// {
	// 	ROS_INFO_STREAM("CHECK");
	// }
	// ROS_INFO_STREAM(log.msg.find("Fail to write"));
	if (log.msg.find("Fail to write") != std::string::npos)
	{	

		// system("bash -c 'source  /opt/ros/kinetic/setup.bash'; rosnode kill gripper_dynamixel");
		// sleep(5);
		// system("bash -c 'source  /opt/ros/kinetic/setup.bash'; roslaunch marco_launcher gripper_dynamixel.launch");

		/*
		ROS_INFO_STREAM("FAIL TO WRITE ERROR DETECTED");

		system("bash -c 'source  /opt/ros/indigo/setup.bash'; rosnode kill gripper_dynamixel");
		sleep(5);
		system("gnome-terminal -x sh -c 'source /opt/ros/indigo/setup.bash; roslaunch marco_launcher gripper_dynamixel.launch'");
		sleep(5);
		*/
		
		ROS_INFO_STREAM("FAIL TO WRITE ERROR DETECTED");

		system("bash -c 'source  /opt/ros/kinetic/setup.bash'; rosnode kill test_node");
		sleep(5);
		system("gnome-terminal -x sh -c 'source /opt/ros/kinetic/setup.bash; rosrun autonomous_pick_place test_node'");
		sleep(5);
		

	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gripper_dynamixel_relauncher");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/rosout_agg",1000,callback);

	ros::spin();
	// while (ros::ok)
	// {
	// 	ros::spin();
	// }
}
