#include <ros/ros.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <sensor_msgs/JointState.h>

ros::Publisher pub;

void callback(const sensor_msgs::JointState::ConstPtr& msg){

  sensor_msgs::JointState new_msg = *msg;
  for(size_t i=0; i<new_msg.position.size(); ++i){
    if(std::isnan(new_msg.position[i])){
      new_msg.position[i] = 0;
    }
  }

  pub.publish(new_msg);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "joint_state_relay_without_nans");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/joint_torque_states", 1000, callback);
  pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  ros::spin();

  return 0;
}
