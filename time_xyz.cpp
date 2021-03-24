#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

ros::Publisher chatter_pub;
nav_msgs::Odometry t;
geometry_msgs::Pose drone_model_pose;

void chatterCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    drone_model_pose = msg->pose[2];

    t.header.stamp = ros::Time::now();
    t.pose.pose.position.x = drone_model_pose.position.x;
    t.pose.pose.position.y = drone_model_pose.position.y;
    t.pose.pose.position.z = drone_model_pose.position.z;

  chatter_pub.publish(t);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_and_pub_node");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1, chatterCallback);

  chatter_pub = nh.advertise<nav_msgs::Odometry>("time_xyz", 1);

  ros::spin();

  return 0;
}