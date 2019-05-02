#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;

	while (ros::ok())
	{
		ROS_INFO_STREAM("STILL RUNNING");
	}

	return 0;
}