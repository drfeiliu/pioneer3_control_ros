#include "AriaRobot.h"

int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria");
  ros::NodeHandle n(std::string("~"));
  Aria::init();
  RosAriaNode *node = new RosAriaNode(n);


  if( node->Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
    return -1;
  }
  //node->robotWander(); 
  //node->robotDemo();
  ROS_INFO( "RosAria: Running... \n" );

  node->spin();
  //ros::Rate loop_rate(0.5);
/*  
  while (ros::ok())
 {

    ROS_INFO("I AM RUNNING\n");
    loop_rate.sleep();
    ros::spinOnce();
  }
*/
  delete node;

  ROS_INFO( "RosAria: Quitting... \n" );
  return 0;
 
}
