#include <ros/ros.h>
//#include "Package/MessageType.h"
#include "geometry_msgs/Twist.h"
//using geometry_msgs::Twist;
//using namespace std;

const double TWIST_LINEAR = 0.5; //.5 m/s forward
const double TWIST_ANGULAR = 0.2; //0 rad/s 
int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_cmd");
    ros::NodeHandle n;
    //ros::Publisher pkgName = nodeName.advertise<Package::MessageType>("/topicName", 100);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 100);//

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = TWIST_LINEAR;
        cmd_msg.angular.z = TWIST_ANGULAR;

        cmd_pub.publish(cmd_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
