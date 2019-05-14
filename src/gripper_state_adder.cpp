#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <dynamixel_workbench_msgs/DynamixelXM.h>
#include <math.h>


double deg_to_rad(double deg)
{

  return (double) deg * M_PI / 180;
}

double max_steps = 2000;//1400;
double min_steps = 200;//0;
double deg_range = 73;

// 1281 = open (60deg), -308 = closed (-13deg), 1589 steps = 73 deg
// when torque is enabled: open=0 closed = 1393
double factor_deg_per_step =  deg_range / max_steps;
double factor_step_per_deg =  max_steps / deg_range;
double deg;

double zero_point = (max_steps - factor_step_per_deg * (double) 12);

double factor = (max_steps - zero_point) / max_steps;

// mapping from motor steps to radians
double steps_to_rad(double steps)
{

  // open and close grippper with intermediate states

  /*
  if (steps > zero_point)
  {
    deg =  - factor * steps * factor_deg_per_step;
  }

  else if (steps <= zero_point && steps >= 1)
  {
    deg = (zero_point - steps) * factor_deg_per_step;
  } 
  else if (steps < 1)
  {
    deg = 60;
  }
 */
  

  // open and close gripper with only 2 states, open and close
  // double deg;
  // old box
/*
  if (steps > 1000)
  {
    deg = -13;
  }
  else if (steps <= 1000)
  {
    deg = 60;
  }
*/

  // new bolletje box, need to change the position at which the gripper should be open in simulation
  if (steps > 1200)
  {
    deg = -13;
  }
  else if (steps <= 1200)
  {
    deg = 60;
  }


  
  //ROS_INFO_STREAM("\t" << deg);
  return deg_to_rad(deg);
}

class GripperStateNode
{

  public:
      double gripper_position;
      sensor_msgs::JointState arm_torso_gripper_joints;
      ros::NodeHandle n;
      ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/joint_states_with_gripper", 1000);

      void gripper_state_callback(const dynamixel_workbench_msgs::DynamixelXM& msg)
      {
        //ROS_INFO_STREAM("I heard:" << steps_to_rad(msg.present_position));
        this->gripper_position = steps_to_rad(msg.present_position);
      }

      void arm_torso_state_callback(const sensor_msgs::JointState& msg)
      {
        //ROS_INFO_STREAM("I heard:" << steps_to_rad(msg.present_position));
        this->arm_torso_gripper_joints = msg;
        this->arm_torso_gripper_joints.name.push_back("gripper_joint");


        this->arm_torso_gripper_joints.position.push_back(this->gripper_position);
        this->arm_torso_gripper_joints.velocity.push_back(0.0);
        this->arm_torso_gripper_joints.effort.push_back(0.0);

        pub.publish(arm_torso_gripper_joints);


      }

      void add_gripper_state()
      {
        ros::Subscriber sub1 = n.subscribe("/dynamixel_workbench_single_manager/motor_state", 1000, &GripperStateNode::gripper_state_callback, this);
        ros::Subscriber sub2 = n.subscribe("/joint_states", 1000, &GripperStateNode::arm_torso_state_callback, this);
        

        // 

        ros::spin();
        
      }
};
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "gripper_state_adder");
  GripperStateNode gripper_state_adder;

  gripper_state_adder.add_gripper_state();
  
  

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  
 

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */



  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  

  return 0;
}