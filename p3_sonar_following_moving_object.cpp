/***********************************************
   Date:      07, April, 2014
   Author:    Fei Liu
   File:      p3_sonar_following_moving_object.cpp
   Function:  Read (subscribe) sonar data and show it, and 
              follow a moving object
   Modification:
           1) 08,May,2014 by Fei Liu
              the comments are renewed  
***********************************************/


/*         sensor_msgs/PointCloud
# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
*/

/* example of output
http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
channels: []
---
header: 
  seq: 143
  stamp: 
    secs: 1396372374
    nsecs: 16548440
  frame_id: sonar_frame
points: 

  -1 
    x: 0.0689999982715   //coordination X
    y: 1.19500005245     //coordination Y
    z: 0.0
  -2 
    x: 1.27423167229
    y: 1.50171017647
    z: 0.0
  -3 
    x: 1.08330738544
    y: 0.617999970913
    z: 0.0
  -4 
    x: 1.09762811661
    y: 0.191271170974
    z: 0.0
  -5 
    x: 1.11043059826
    y: -0.193528607488
    z: 0.0
  -6 
    x: 1.12487661839
    y: -0.64200001955
    z: 0.0
  -7 
    x: 1.00040411949
    y: -1.17537534237
    z: 0.0
  -8 
    x: 0.0689999982715
    y: -2.32699990273
    z: 0.0
  -9 
    x: -0.157000005245
    y: -2.35500001907
    z: 0.0
  -10 
    x: -0.578387975693
    y: -0.566369950771
    z: 0.0
  -11 
    x: -0.587740302086
    y: -0.280499994755
    z: 0.0
  -12 
    x: -2.06704616547
    y: -0.346512645483
    z: 0.0
  -13 
    x: -2.0700006485
    y: 0.347033590078
    z: 0.0
  -14 
    x: -1.01555681229
    y: 0.527499973774
    z: 0.0
  -15 
    x: -1.01355516911
    y: 1.08498203754
    z: 0.0
  -16 
    x: -0.157000005245
    y: 1.18700003624
    z: 0.0

*/

/*   reference
http://stanford-ros-pkg.googlecode.com/svn-history/r146/trunk/recyclerbot/src/object_detector_node.cpp

*/

/*******about the rays and distance*******/

//               front sonar beams
//                      3 4            +/-10 degree
//                   2   ^   5         +/-30 degree
//                 1     |     6       +/-50 degree
//             ___0___center____7____  +/-90 degree
//             |         |          |
//             |         |          |
//             |         |          |
//             |      P3_ROBOT      |
//             |         |          |
//             |         |          |
//             |         |          |
//             |         |          |
//             |         |          |
//             |_________|__________|

// distance offset(mm):
//rays no.: 0   1   2   3   4   5   6   7
//offset  :160 220 240 240 240 240 220 160
/*******about the rays and distance*******/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>   //for sonar data
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <iostream>

#define DEBUG_PRINT

using geometry_msgs::Twist;
using namespace std;

Twist vel;

#define SONAR_NUM 8
const float PI = 3.1416;

const int OFFSET[SONAR_NUM] ={160, 220, 240, 240, 240, 240, 220, 160};
//angles of rays to heading
const float ANGLE_RAY[SONAR_NUM] = {(-1.0/2)*PI, (-5.0/18)*PI, (-1.0/6)*PI, (-1.0/18)*PI, (1.0/18)*PI, (1.0/6)*PI, (5.0/18)*PI, (1.0/2)*PI};


//distance to the obstacle in front of the sonar ray
int distToObstace[SONAR_NUM];

//desired safe distance to object (mm)
const int DIST_DESIRED = 300; 
//the obstacle within this distance(mm) is recognized as a object
const int DIST_EFFECT = 1000;

const float HEADING_VEL_MAX = 0.50;
const float HEADING_VEL_MIN = -0.300;
const float ANGULAR_VEL_MAX = PI/4.0;
const float ANGULAR_VEL_MIN = PI/(-4.0);

//The time desired to catch up with the object
const float TIME_DESIRED = 1.2;
//show that if any object is detected
bool ifObjectDetected = false;

//the heading velocity to follow the object
float heading_vel = 0.0; 
//the angular velocity to follow the object
float angular_vel = 0.0;

//the angle between the object and the robot's heading
float angle_error_to_head = 0;
//the object's direction
//not used
//int objection_direction = 0;

//which ray detectes the object with te minimum distane
int ray_seq_min_dist = 3;
//the minimum distane to the object detected
int dist_to_obj_min = 0;

/***********************************
Function   : CheckIfObjectDetected
Description:
              Check if any object is detected,
              if any object is detected, return (true)
              or return (false)
***********************************/
bool CheckIfObjectDetected()
{
    int ray_seq = 1;
    
    while(ray_seq <= 6)
    {
         if (!(distToObstace[ray_seq] > DIST_EFFECT))
         {//if one ray detects an object in effective area
             return true;
         }
         else
         {
             ray_seq++;
         }
    }
    //if no object is in effective area
    return false;

}


/***********************************
Function   : CalculateDistaneAndHeading
Description:
             Calculate the distace between the robot
              and the object, and determine if the object
              is on the left/right or in front the robot
***********************************/
void CalculateDistaneAndHeading()
{
    dist_to_obj_min = distToObstace[1];

    /*get the direction and minimum distance*/
    for (int ray_seq=2; ray_seq<=6; ray_seq++)
    {//discard ray_0 and ray_6
        if (distToObstace[ray_seq] < dist_to_obj_min)
        {
            dist_to_obj_min = distToObstace[ray_seq];
            ray_seq_min_dist = ray_seq;
        }
    }
#ifdef DEBUG_PRINT
    printf("dist_to_obj_min is %d\n", dist_to_obj_min);
#endif
    /*get the angular between HEADING AND OBJECT*/
    if (ray_seq_min_dist ==3 || ray_seq_min_dist ==4)
    {//the object is just ahead        
        angle_error_to_head = 0;
    }
    else if (ray_seq_min_dist ==1 || ray_seq_min_dist ==2)
    {//the object is on the left        
        angle_error_to_head = (ANGLE_RAY[1] + ANGLE_RAY[2])/2;
    }
    else if (ray_seq_min_dist ==5 || ray_seq_min_dist ==6)
    {//the object is on the right        
        angle_error_to_head = (ANGLE_RAY[5] + ANGLE_RAY[6])/2;
    }
    else
    {
        //we donot consider the object is verticle to heading,
        //which means
        //(ray_seq_min_dist ==0 || ray_seq_min_dist ==8)
    }
#ifdef DEBUG_PRINT
    printf("Angle_error_to_head is %f\n", angle_error_to_head);
#endif
}

/***********************************
Function   : Heading_Vel_Determination
Description:
             Determine the heading velocity
***********************************/
void Heading_Vel_Determination()
{
    heading_vel =((dist_to_obj_min - DIST_DESIRED)*1.0/1000)/TIME_DESIRED;
    if (heading_vel > 0)
#ifdef DEBUG_PRINT
        printf("Too far!\n");
#endif
    else if (heading_vel < 0)
#ifdef DEBUG_PRINT
        printf("Too close!\n");
#endif
    //printf(" %d - %d / %d /1000 = %f\n",dist_to_obj_min, DIST_DESIRED, TIME_DESIRED, heading_vel);
#ifdef DEBUG_PRINT
    printf("heading_vel = %f\n", heading_vel);
#endif
}


/***********************************
Function   : Angular_Vel_Determination
Description:
             Determine the angular velocity
***********************************/
void Angular_Vel_Determination()
{
    angular_vel = -1.0*angle_error_to_head / TIME_DESIRED;
#ifdef DEBUG_PRINT
    printf("angular_vel = %f\n", angular_vel);
#endif
}


void FollowingObject()
{
    ifObjectDetected = CheckIfObjectDetected();
   
    if (ifObjectDetected)
    {//if object detected
#ifdef DEBUG_PRINT
        printf("Object found!\n");
#endif

        CalculateDistaneAndHeading();
        Heading_Vel_Determination();
        Angular_Vel_Determination();
  
        /*Make sure the heading vel is between 
          HEADING_VEL_MIN and HEADING_VEL_MAX*/
        if (heading_vel > HEADING_VEL_MAX)
        {
            heading_vel =  HEADING_VEL_MAX;
        }
        else if (heading_vel <  HEADING_VEL_MIN)
        {
            heading_vel =  HEADING_VEL_MIN;  
        }

        /*Make sure the angular vel is between 
          ANGULAR_VEL_MIN and ANGULAR_VEL_MAX*/
        if (angular_vel > ANGULAR_VEL_MAX)
        {
            angular_vel = ANGULAR_VEL_MAX;
        }
        else if (angular_vel < ANGULAR_VEL_MIN)
        {
            angular_vel = ANGULAR_VEL_MIN;
        }
 
        /*for publishing the vel*/
        vel.linear.x = heading_vel;
        vel.angular.z  = angular_vel;
    }
    else
    {//if object disappears, stop
        vel.linear.x = 0;
        vel.angular.z  = 0;
#ifdef DEBUG_PRINT
        printf("The object disappears!\n");
#endif
    }
#ifdef DEBUG_PRINT
    printf("vel.linear.x = %f, vel.angular.z = %f\n",vel.linear.x, vel.angular.z);
#endif
}

/***********************************
Function   : Get_sonarData_Callback
Description:
             the callback function which deals with getting sonar data
***********************************/
void Get_sonarData_Callback(const sensor_msgs::PointCloud::ConstPtr &sonarScanedData)
{
    //sensor_msgs::PointCloud sonarData;
    int seq = 0;    
    float tmpX = 0.0, tmpY=0.0;
    seq = sonarScanedData->header.seq;
#ifdef DEBUG_PRINT
    printf("seq of sonar beam  and  distance measured-->\n");
    printf("Frame[%d] :  \n", seq);
#endif

#ifdef DEBUG_PRINT
    for (int i=0; i<SONAR_NUM; i++)    
    { 
        printf("%d\t",i);
    }
    printf("\n");
#endif
    for (int i=0; i<SONAR_NUM; i++)    
    {
        tmpX= sonarScanedData->points[i].x; //x coordinate
        tmpY= sonarScanedData->points[i].y; //y coordinate
        distToObstace[i] = int(sqrt(tmpX*tmpX+tmpY*tmpY)*1000-OFFSET[i]);
#ifdef DEBUG_PRINT
        printf("%d\t",distToObstace[i]);
#endif
    }
#ifdef DEBUG_PRINT
    printf("\n\n");
#endif
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "p3_sonar");
  ros::NodeHandle n;

  ros::Publisher vel_pub;
  vel_pub = n.advertise<Twist>("/RosAria/cmd_vel", 1);

  // please modify the topic name "/RosAria_p3at_1_113/sonar" according to the robot that are connected
  ros::Subscriber get_sonar_data = n.subscribe<sensor_msgs::PointCloud>("/RosAria_p3at_1_113/sonar", 100, Get_sonarData_Callback);
#ifdef DEBUG_PRINT
  printf("\n********** Sonar Readings: **********\n");
#endif
  while (ros::ok())
  {  
               
      ros::Duration(0.4).sleep();          
      ros::spinOnce();

      //execute following the object
      FollowingObject();
      vel_pub.publish(vel);
      
  }
  return 0;
}
