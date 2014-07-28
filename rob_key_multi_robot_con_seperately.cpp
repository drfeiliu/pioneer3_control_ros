/**************************************************************************
   Date:      16, May, 2014
   Author:    Fei Liu
   
   File:      rob_key_multi_robot_con_seperately.cpp
   Function:  Control multiple robots by keyboard.
   Description:
            1. Choose robot:
               Press Q to select all robots or which robot to control
            2. control the robot(s)
               1) Press Up Arrow to move ahead
               2) Press Left Arrow to turn left
               3) Press Right Arrow to turn right
               4) Press Down Arrow to move back
               5) Press Space to stop moving
       
   Modification:
           1) 16, May, 2014 by Fei
               add the function that choosing robot to control
**************************************************************************/





#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include <termios.h>

//Key code to use
#define KEYCODE_RIGHT       0x43//  -->
#define KEYCODE_LEFT        0x44//  <--
#define KEYCODE_UP          0x41//   ^
#define KEYCODE_DOWN        0x42//  \|/

#define KEYCODE_Q           0x71//   q 
#define KEYCODE_S           0x73//   s
#define KEYCODE_T           0x74//   t 
#define KEYCODE_U           0x75//   u
#define KEYCODE_V           0x76//   v 
#define KEYCODE_W           0x77//   w
#define KEYCODE_X           0x78//   x 
#define KEYCODE_Y           0x79//   x 

#define KEYCODE_EXCLAMATION 0X21//   !
#define KEYCODE_RETURN      0X0D//   Enter
#define KEY_SPACE           0x20//   SPACE

#define KEY_ZERO            0x30//    0
#define KEY_ONE             0x31//    1
#define KEY_TWO             0x32//    2
#define KEY_THREE           0x33//    3
#define KEY_FOUR            0x34//    4
#define KEY_FIVE            0x35//    5

using geometry_msgs::Twist;
using namespace std;

ros::Publisher vel_pub_all;
ros::Publisher vel_pub_p0_119;
ros::Publisher vel_pub_p1_113;
ros::Publisher vel_pub_p2_246;
ros::Publisher vel_pub_p3_191;
ros::Publisher vel_pub_p4_254;
ros::Publisher vel_pub_p5_160;
 
ros::Time t1;

//different velocities for different robots
Twist vel;//for all the robots to control
Twist vel_static;
Twist vel_robot_p0_119;//for robot with IP 192.168.0.119(LOCAL laptop-simulation)
Twist vel_robot_p1_113;//for robot with IP 192.168.0.113
Twist vel_robot_p2_246;//for robot with IP 192.168.0.246
Twist vel_robot_p3_191;//for robot with IP 192.168.0.191
Twist vel_robot_p4_254;//for robot with IP 192.168.0.254
Twist vel_robot_p5_160;//for robot with IP 192.168.0.160


//select the robot to control
//IF 0 is selected, control all the robots
#define ROBOT_NUM 6
const unsigned int IP_LAST4NUM[ROBOT_NUM+1]={119, 113, 246, 191, 254, 160, 0};
unsigned int robot_select_seq = 0;
unsigned int robot_selected_IP = 0;
bool if_selecting_robot = false;
bool if_selected[ROBOT_NUM+1];// = {false,false,false,false,false,false,false};

int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
   {
     tcsetattr(kfd, TCSANOW, &cooked);
     ros::shutdown();
     exit(0);
   }


int main(int argc, char** argv)
{
   ros::init(argc, argv, "rob_key_multi_robot_con_seperately");
   ros::NodeHandle n;
   vel_pub_all = n.advertise<Twist>("/RosAria/cmd_vel", 1);
   vel_pub_p0_119 = n.advertise<Twist>("/RosAria/cmd_vel_p0_119", 1);
   vel_pub_p1_113 = n.advertise<Twist>("/RosAria/cmd_vel_p1_113", 1);
   vel_pub_p2_246 = n.advertise<Twist>("/RosAria/cmd_vel_p2_246", 1);
   vel_pub_p3_191 = n.advertise<Twist>("/RosAria/cmd_vel_p3_191", 1);
   vel_pub_p4_254 = n.advertise<Twist>("/RosAria/cmd_vel_p4_254", 10);
   vel_pub_p5_160 = n.advertise<Twist>("/RosAria/cmd_vel_p5_160", 1);

     
   for (int i=0; i<=ROBOT_NUM; i++)
   {
       if_selected[i] = false;
   }
   signal(SIGINT,quit);
   char c;
   bool is_pub_vel=false;

   t1=ros::Time::now();

   tcgetattr(kfd, &cooked);
   memcpy(&raw, &cooked, sizeof(struct termios));
   raw.c_lflag &=~ (ICANON | ECHO);
   raw.c_cc[VEOL] = 1;
   raw.c_cc[VEOF] = 2;
   tcsetattr(kfd, TCSANOW, &raw);

   vel_static.linear.x=0;
   vel_static.angular.z = 0;
   puts("\n*************************************************\nNow, All the robots can NOT be controlled. \nPlease Press q or s to select the robot to control:\n\n*************************************************\n*\t\tP0:192.168.0.119\t\t*\n*\t\tP1:192.168.0.113\t\t*\n*\t\tP2:192.168.0.246\t\t*\n*\t\tP3:192.168.0.191\t\t*\n*\t\tP4:192.168.0.254\t\t*\n*\t\tP5:192.168.0.160\t\t*\n*************************************************\n");

 while (ros::ok())
  {       
     if(read(kfd, &c, 1) < 0)
     {
              perror("read():");
         exit(-1);
     }

     switch(c)
     {
         case KEYCODE_LEFT://turn left
             ROS_DEBUG("LEFT");
             puts("TURN LEFT");
             vel.linear.x = 0.0;
             vel.angular.z  = 0.1;
             printf("vel.inear.x=%f, vel.angular.z=%f\n", vel.linear.x, vel.angular.z);
             is_pub_vel = true;
             break;

         case KEYCODE_RIGHT://turn right
             ROS_DEBUG("RIGHT");
             puts("TURN RIGHT");
             vel.linear.x = 0.0;
             vel.angular.z  = -0.1;
             printf("vel.inear.x=%f, vel.angular.z=%f\n", vel.linear.x, vel.angular.z);
             is_pub_vel = true;
             break;

         case KEYCODE_UP://move forward
             ROS_DEBUG("UP");
             puts("FORWARD");
             vel.linear.x = 0.1;
             vel.angular.z  = 0;
             printf("vel.inear.x=%f, vel.angular.z=%f\n", vel.linear.x, vel.angular.z);
             is_pub_vel = true;
             break;

         case KEYCODE_DOWN://move backward
                ROS_DEBUG("DOWN");
                puts("BACKWARD");
                vel.linear.x = -0.1;
                vel.angular.z  = 0;
             printf("vel.inear.x=%f, vel.angular.z=%f\n", vel.linear.x, vel.angular.z);
                ROS_DEBUG("LEFT");
             is_pub_vel = true;
             break;

         case KEY_SPACE://stop
             ROS_DEBUG("STOP");
             puts("SPACE");
             vel.linear.x = 0.0;
             vel.angular.z = 0.0;
             printf("vel.inear.x=%f, vel.angular.z=%f\n", vel.linear.x, vel.angular.z);
             is_pub_vel = true;
             break;
         case KEYCODE_Q://Choose the robot sequently
             puts("===============\nq is pressed\n===============");
             puts("continuing press q to change the robot to control:");
             vel.linear.x = 0.0;
             vel.angular.z = 0.0;
             if (robot_select_seq < ROBOT_NUM)
             {
                 for (int i=0; i<=ROBOT_NUM; i++)
                 {
                     if (i!=robot_select_seq)
                         if_selected[i]=false;
                     else
                         if_selected[i]=true;
                 }
                 robot_selected_IP = IP_LAST4NUM[robot_select_seq];
                 printf("the robot P[%d] with IP address 192.168.0.%d is selected to control!\n", robot_select_seq, IP_LAST4NUM[robot_select_seq]);

                 robot_select_seq ++;
             }
             else
             {
                 robot_select_seq = ROBOT_NUM;
                 for (int i=0; i<=ROBOT_NUM; i++)
                 {
                     if (i==ROBOT_NUM)
                     {
                         if_selected[i]=true;
                     }
                     else
                     {
                         if_selected[i]=false;
                     }
                 }
                 robot_selected_IP=IP_LAST4NUM[robot_select_seq];
                 puts("Now all the 5 robots can be controlled!");
             }
             is_pub_vel = true;
             break;

             //Select the robots by inputting the number of the robot
             case KEYCODE_S:    
                 //stop all the robots first
                 vel_pub_all.publish(vel_static);
                 puts("===============\ns is pressed\n===============");

                 for (int i=0; i<=ROBOT_NUM; i++)
                 {
                    if_selected[i]=false;
                 }
                 printf("Please Input the number of the robots you want to control simultaneously!\n");
                 printf("\n*****************************************\n*\t0 - P0:192.168.0.119\t\t*\n*\t1 - P1:192.168.0.113\t\t*\n*\t2 - P2:192.168.0.246\t\t*\n*\t3 - P3:192.168.0.191\t\t*\n*\t4 - P4:192.168.0.254\t\t*\n*\t5 - P5:192.168.0.160\t\t*\n*****************************************\n");
                 printf("If you entered the wrong number, please repress s to select again.\nAfter you finish your selection, Press y to ensure!\n");
                 printf("The robots you selected are:\n");
                 if_selecting_robot = true;
                 break;
             case KEY_ZERO://KEYCODE_T://
                 //puts("p1");//printf("p1\t");
                 if (if_selecting_robot)
                 {
                     if_selected[0] = true;
                     puts("0-LOCAL LAPTOP-SIMULATION ROBOT");
                 }
                 break;
             case KEY_ONE://KEYCODE_T://
                 //puts("p1");//printf("p1\t");
                 if (if_selecting_robot)
                 {
                     if_selected[1] = true;
                     puts("1-MARHES LAB-p3at-p1:192.168.0.113");
                 }
                 break;
             case KEY_TWO://KEYCODE_U://
                 if (if_selecting_robot)
                 {
                     if_selected[2] = true;
                     puts("2-MARHES LAB-p3at-p2:192.168.0.246");
                 }
                 break;
             case KEY_THREE://KEYCODE_V://
                 if (if_selecting_robot)
                 {
                     if_selected[3] = true;
                     puts("3-MARHES LAB-p3at-p3:192.168.0.191");
                 }
                 break;
             case KEY_FOUR://KEYCODE_W://
                 if (if_selecting_robot)
                 {
                     if_selected[4] = true;
                     puts("4-MARHES LAB-p3at-p4:192.168.0.254");
                 }
                 break;
             case KEY_FIVE://KEYCODE_X://
                 if (if_selecting_robot)
                 {
                     if_selected[5] = true;
                     puts("5-MARHES LAB-p3at-p5:192.168.0.160");
                 }
                 break;
             case KEYCODE_Y://KEYCODE_RETURN:
                 if(if_selecting_robot)
                 {
                     vel.linear.x = 0;
                     vel.angular.z = 0;
                     if_selecting_robot = false;
                     printf("\nNow you can control the robots you selected!\n");
                 }
                 is_pub_vel=true;
                 break;

       }//end of switch(c)
          
       if(is_pub_vel==true)
       {
           vel_pub_all.publish(vel_static);
           //puts("clean!");

           if(if_selected[0]&&if_selected[1]&&if_selected[2]&&if_selected[3]&&if_selected[4]&&if_selected[5])
           {
               if_selected[ROBOT_NUM] = true;
           }

           if(if_selected[ROBOT_NUM])
           {
               vel_pub_all.publish(vel);
               puts("For All marhes's p3at and local robot in simulation:");
           }
           else
           { 
             if(if_selected[0])
             {
               vel_robot_p0_119.linear.x = vel.linear.x;
               vel_robot_p0_119.angular.z = vel.angular.z;
               vel_pub_p0_119.publish(vel_robot_p0_119);
               puts("For Robot P0(192.168.0.119)(local simulation robot):");
             }

             if(if_selected[1])
             {
               vel_robot_p1_113.linear.x = vel.linear.x;
               vel_robot_p1_113.angular.z = vel.angular.z;
               vel_pub_p1_113.publish(vel_robot_p1_113);
               puts("For Robot P1(192.168.0.113):");
             }
 
             if(if_selected[2])
             {
               vel_robot_p2_246.linear.x = vel.linear.x;
               vel_robot_p2_246.angular.z = vel.angular.z;
               vel_pub_p2_246.publish(vel_robot_p2_246);
               puts("For Robot P2(192.168.0.246):");
             }

             if(if_selected[3])
             {
                   vel_robot_p3_191.linear.x = vel.linear.x;
                   vel_robot_p3_191.angular.z = vel.angular.z;
                   vel_pub_p3_191.publish(vel_robot_p3_191);
                   puts("For Robot P3(192.168.0.191):");
             }    

             if(if_selected[4])
             {
                   vel_robot_p4_254.linear.x = vel.linear.x;
                   vel_robot_p4_254.angular.z = vel.angular.z;
                   vel_pub_p4_254.publish(vel_robot_p4_254);
                   puts("For Robot P4(192.168.0.254):");
                   //printf("vel is linear vel=%f, angular vel=%f\n", vel_robot_p4_254.linear.x, vel_robot_p4_254.angular.z);
             }

             if(if_selected[5])
             {
                   vel_robot_p5_160.linear.x = vel.linear.x;
                   vel_robot_p5_160.angular.z = vel.angular.z;
                   vel_pub_p5_160.publish(vel_robot_p5_160);
                   puts("For Robot P5(192.168.0.160):");
             }
           }//end of else
             
           is_pub_vel=false;
       }//end of if(is_pub_vel==true)
       
    ros::spinOnce();
   
  }//while
return(0);
}


