#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int add_marker_to_cup(int argc, char** argv){

  char** argv = {};
  int argc = 0;

  ros::init(argc, argv, "frame_adder");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(-0.625-0.0927, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
    rate.sleep();
  }
  return 0;
};
