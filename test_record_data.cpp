/***********************************************
   Data:      2014-02-06
   Author:    Fei Liu
   File:      tet_record_data.cpp
   Function:  Read (subscribe) data and show it
  
***********************************************/
#include <ros/ros.h>
//#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

void get_poseCallback(const nav_msgs::Odometry::ConstPtr& poseData)
 {
    double xPos=poseData->pose.pose.position.x;
    ROS_INFO("x_pose is [%f]\t", xPos);
    double yPos=poseData->pose.pose.position.y;
    ROS_INFO("y_pose is [%f]\t", yPos);
    //get Quaternion anglular information
    double x_Ori=poseData->pose.pose.orientation.x;
    ROS_INFO("x_Ori is: [%f]", x_Ori);
    double y_Ori=poseData->pose.pose.orientation.y;
    ROS_INFO("y_Ori is: [%f]", y_Ori);
    double z_Ori=poseData->pose.pose.orientation.z;
    ROS_INFO("z_Ori is: [%f]\n", z_Ori);

    ROS_INFO("\n");
 }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_record_data");
  ros::NodeHandle n;

  //cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function <void(const   geometry_msgs::TwistConstPtr&)>)
  //boost::bind(&RosAriaNode::cmdvel_cb, this, _1 ));
  ros::Subscriber get_pose = n.subscribe<nav_msgs::Odometry>("/RosAria_local/pose", 1000, get_poseCallback);
  ROS_INFO("I heard the pose is: \n");

  while (ros::ok())
  {  
               
      ros::Duration(1).sleep(); 
         
      ros::spinOnce();
  }
}
