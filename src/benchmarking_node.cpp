#include <ros/ros.h>
#include "benchmarking.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "motion_planners_benchmarking_node");
	ros::AsyncSpinner spinner(1);
	Benchmarking benchmarking;

	while(ros::ok)
	{
		//ros::spinOnce();
		spinner.start();
		sleep(1);
		spinner.stop();
	}

	return 0;
}