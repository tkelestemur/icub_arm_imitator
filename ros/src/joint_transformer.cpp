#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "joint_transformer");
  ros::NodeHandle node;
  ros::Publisher jointPosePub = node.advertise<geometry_msgs::Pose>("icub/jointPose", 1000);

  string target_frame = "/HandRight"; // right hand
  string ref_frame = "/SpineBase"; // torso base

//   string target_frame = "/joint_11"; // right hand
// string ref_frame = "/joint_0"; // torso base

  geometry_msgs::Pose jointPose;
  tf::TransformListener listener;
  // ofstream myfile;
  // myfile.open("tf_handtotorso.csv");


  ros::Rate rate(30.0);

  int count = 0;
  while (node.ok()){

    std::stringstream ss;
    ss << count;
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(ref_frame, target_frame, ros::Time(0), transform);
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
    // std::cout << "x: " << jointPose.position.x << " y: " << jointPose.position.y <<" z: " << jointPose.position.z <<std::endl;
    // myfile << jointPose.position.x <<" , "
    //        << jointPose.position.y <<" , "
    //        << jointPose.position.z<<" , ";
    // myfile << endl;

    jointPosePub.publish(jointPose);
    rate.sleep();
  }
  return 0;
};
