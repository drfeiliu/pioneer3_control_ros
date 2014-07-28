/***********************************************
   Data:      2014-04-01
   Author:    Fei Liu
   File:      p3_sonar.cpp
   Function:  Read (subscribe) sonar data and display it on screen
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


/* example of output(using rostopic)
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
//0   1   2   3   4   5   6   7
//160 220 240 240 240 240 220 160
/*******about the rays and distance*******/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>   //for sonar data
#include <math.h>
#include <iostream>

using namespace std;

//in this case, we just use 8 sonars that are in front of the robot
#define SONAR_NUM 8

int offset[SONAR_NUM] ={160, 220, 240, 240, 240, 240, 220, 160};


//sensor_msgs::PointCloud sonarData;
int seq = 0;

int distToObstace[SONAR_NUM];

void get_sonarData_Callback(const sensor_msgs::PointCloud::ConstPtr &sonarScanedData)
{
    float tmpX = 0.0, tmpY=0.0;
    seq = sonarScanedData->header.seq;

    printf("seq of sonar beam  and  distance measured-->\n");
    printf("Frame[%d] :  \n", seq);
    for (int i=0; i<SONAR_NUM; i++)    
    { 
        printf("%d\t",i);
    }
    printf("\n");

    for (int i=0; i<SONAR_NUM; i++)    
    {
        tmpX= sonarScanedData->points[i].x; //coordinate x
        tmpY= sonarScanedData->points[i].y; //coordinate y
        distToObstace[i] = int(sqrt(tmpX*tmpX+tmpY*tmpY)*1000-offset[i]);
        printf("%d\t",distToObstace[i]);
    }
    printf("\n\n");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "p3_sonar");
  ros::NodeHandle n;

  // if it can't get the sonar data, try "rostopic list" to see
  // what actualy the topic is, maybe it will be "/RosAria_local/sonar" which is 
  // mostly in the simulation on MobileSim while in real robot it may be "/RosAria/sonar"
  
  ros::Subscriber get_sonar_data = n.subscribe<sensor_msgs::PointCloud>("RosAria/sonar", 100, get_sonarData_Callback);
  printf("\n********** Sonar Readings: **********\n");

  while (ros::ok())
  {  
               
      ros::Duration(0.2).sleep(); 
         
      ros::spinOnce();
  }
  return 0;
}
