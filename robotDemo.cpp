#include "AriaRobot.h"


int main( int argc, char** argv )
{
  ros::init(argc,argv, "robotDemo");
  ros::NodeHandle n(std::string("~"));
  Aria::init();
  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
    return -1;
  }

  node->robotDemo();
  ROS_INFO( "The robot is under teleoperating now \n\n**************************************************\n\n\t\tPlease Press ESC to STOP! \n\n**************************************************" );

  node->spin();

  delete node;

  ROS_INFO( "RosAria: Quitting... \n" );
  return 0;
 
}
