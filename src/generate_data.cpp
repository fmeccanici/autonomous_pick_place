#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_publisher");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1000);
	ros::Rate loop_rate(10);

	trajectory_msgs::JointTrajectory msg;
	std::vector<std::string> joint_names = {"head_1_joint", "head_2_joint"};



	std::vector<trajectory_msgs::JointTrajectoryPoint> points(1);
	std::vector<double> positions = {-0.4, -0.2};



	while(ros::ok())
	{

		points[0].positions = positions;
		points[0].time_from_start = ros::Duration(1);

		msg.joint_names = joint_names;
		msg.points = points;

		//ROS_INFO_STREAM("\t" << msg);
		pub.publish(msg);

		ros::spinOnce();
		positions[0] += 0.01; 
		sleep(5);

		if (positions[1] >= 0.1)
		{
			break;
		}
	}

	return 0;
}

