/*****************************************************
Date      : Wed., 19/02/14
Author    : Fei Liu
File Name : robWander.cpp
Function  : the robot will wander in an obstacle placed
            environment. It can avoid obstacle. Press 
            "ESC" to stop.
*****************************************************/

#include "AriaRobot.h"


int main( int argc, char** argv )
{
  ros::init(argc,argv, "robotWander");
  ros::NodeHandle n(std::string("~"));
  Aria::init();
  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
    return -1;
  }
  ROS_INFO( "\nThe robot is wandering now!\n\n**************************************************\n\n\t\tPlease Press ESC to STOP! \n\n**************************************************" );

  //node->spin();
  node->robotWander(); 
  delete node;

  ROS_INFO( "RosAria: Quitting... \n" );
  return 0;
}
