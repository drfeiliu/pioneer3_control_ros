/**************************************************************************
   Date:      16, Feb., 2014
   File:      android_teleop.cpp
   Function:  use your Android phone to control the robot 
   Description: 
              1) http://wiki.ros.org/ROSARIA/Tutorials/iPhone%20Teleop%20with%20ROSARIA/Android%20teleop%20Pioneer%203at
              2) you need to download ROS Sensor Drive to your phone
**************************************************************************/

   #include <ros/ros.h>
   #include "geometry_msgs/Twist.h"
   #include "rosaria/BumperState.h"
   #include "sensor_msgs/Imu.h"
   #include <signal.h>
   #include <termios.h>
   
   
   using geometry_msgs::Twist;
   using namespace std;
   
   ros::Publisher vel_pub;
   ros::Time t1;
   Twist vel;
   int kfd = 0;
   struct termios cooked, raw;
   unsigned int temp=0;
   float x;
   float y;
   float z;
   
   
   void quit(int sig)
   {
       tcsetattr(kfd, TCSANOW, &cooked);
       ros::shutdown();
       exit(0);
   }
   

   void anCallback(const sensor_msgs::Imu::ConstPtr& ansmsg)
   {
       x=ansmsg->angular_velocity.x;
       y=ansmsg->angular_velocity.y;
       z=ansmsg->angular_velocity.z; 
   }


   int main(int argc, char** argv)
   { 
       int ther=3;
   
       ros::init(argc, argv, "android_teleop");
       ros::NodeHandle n;
       vel_pub = n.advertise<Twist>("/RosAria/cmd_vel", 1);  
       signal(SIGINT,quit);
       ros::Rate r(5);

       char c;
       bool dirty=false;   
       t1=ros::Time::now();
   
       tcgetattr(kfd, &cooked);
       memcpy(&raw, &cooked, sizeof(struct termios));
       raw.c_lflag &=~ (ICANON | ECHO);
       raw.c_cc[VEOL] = 1;
       raw.c_cc[VEOF] = 2;
       tcsetattr(kfd, TCSANOW, &raw);
   
       //subscribe to android imu sensor msgs
       ros::Subscriber imu_pub = n.subscribe<sensor_msgs::Imu>("/android/imu", 1, anCallback);
    
   
       while (ros::ok())
       {     
         if(x > ther)
         {
           vel.linear.x = 0.2;
           vel.linear.y=0;
           vel.linear.z=0;
           vel.angular.x = 0;
           vel.angular.y = 0;
           vel.angular.z = 0;
           ROS_INFO("forward");  
         }

         if(x < -ther)
         {
           vel.linear.x = -0.2;
           vel.linear.y=0;
           vel.linear.z=0;
           vel.angular.x = 0;
           vel.angular.y = 0;
           vel.angular.z = 0;
           ROS_INFO("Backward");
         }
       
         if(z > ther)
         {
           vel.linear.x = 0;
           vel.linear.y=0;
           vel.linear.z=0;
           vel.angular.x = 0;
           vel.angular.y = 0;
           vel.angular.z = 0.5;
           ROS_INFO("Turnleft");
         }
 
         if(z < -ther)
         {
           vel.linear.x = 0;
           vel.linear.y=0;
           vel.linear.z=0;
           vel.angular.x = 0;
           vel.angular.y = 0;
           vel.angular.z = -0.5;
           ROS_INFO("Turnright");
         }
 
         if(y < -ther)
         {
           vel.linear.x = 0;
           vel.linear.y=0;
           vel.linear.z=0;
           vel.angular.x = 0;
           vel.angular.y = 0;
           vel.angular.z = 0;
           ROS_INFO("stop");
         }
 
         if(y > ther)
         {
           vel.linear.x = 0;
           vel.linear.y=0;
           vel.linear.z=0;
           vel.angular.x = 0;
           vel.angular.y = 0;
           vel.angular.z = 0;
           ROS_INFO("stop");
         }
         vel_pub.publish(vel);
               
         ros::Duration(0.1).sleep();
         ros::spinOnce();     
    }//while
  
    return(0);
  }
