#include "AriaRobot.h"


int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria_local");
  ros::NodeHandle n(std::string("~"));
  Aria::init();
  RosAriaNode *node = new RosAriaNode(n);

//====================added by Fei===================  
//cmdMovementMode_sub = n.subscribe("move_mode", 100, (boost::function <void (const std_msgs::String&)>)boost::bind(&RosAriaNode::cmdMovementMode_cb, this, _1 ));

//ros::Subscriber move_mode_sub = n.subscribe("move_mode", 100, &RosAriaNode::cmdMovementMode_cb, node);
//====================15/02/2014=====================


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
