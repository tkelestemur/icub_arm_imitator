#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "joint_transformer");
  ros::NodeHandle node;
  ros::Publisher jointPosePub = node.advertise<geometry_msgs::Pose>("icub/jointPose", 1000);

  string target_frame = "/joint_11"; // right hand
  string ref_frame = "/joint_0"; // torso base

  geometry_msgs::Pose jointPose;


  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(target_frame, ref_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    jointPose.position.x = transform.getOrigin().x();
    jointPose.position.y = transform.getOrigin().y();
    jointPose.position.z = transform.getOrigin().z();
    jointPose.orientation.x = transform.getRotation().x();
    jointPose.orientation.y = transform.getRotation().y();
    jointPose.orientation.z = transform.getRotation().z();
    jointPose.orientation.w = transform.getRotation().w();

    ROS_INFO("x =[%f], y =[%f], z =[%f]", jointPose.position.x, jointPose.position.y, jointPose.position.z);
    // ROS_INFO("X =[%f], Y =[%f], Z =[%f], W =[%f]", jointPose.orientation.x, jointPose.orientation.y, jointPose.orientation.z, jointPose.orientation.w);

    jointPosePub.publish(jointPose);
    rate.sleep();
  }
  return 0;
};
