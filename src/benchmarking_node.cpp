#include <ros/ros.h>
#include "benchmarking.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "motion_planners_benchmarking_node");

	Benchmarking benchmarking;

	while(ros::ok)
	{
		ros::spinOnce();
		sleep(1);
	}

	return 0;
}