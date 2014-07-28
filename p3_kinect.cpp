/**************************************************************************
   Date:        26, Feb., 2014
   File:        p3_kinect.cpp
   Function:    to read data from kinect
   Description:
                1) openni_launch
                   http://wiki.ros.org/openni_launch 
                   to start kinect:
                   $ roslaunch openni_launch openni.launch

                2) topics
                   http://mirror.umd.edu/roswiki/openni_launch%282f%29Tutorials%282f%29BagRecordingPlayback.html 
                   http://docs.ros.org/api/sensor_msgs/html/msg/Image.html 

                3) sensor_msg/CameraInfo.msg
                   http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html 

                4) with Matlab
                   https://alliance.seas.upenn.edu/~meam620/wiki/index.php?n=Roslab.KinectMatlab 
                   http://surenkum.blogspot.com/2012/06/getting-kinect-to-work.html     

                5) others
                   http://wiki.ros.org/openni_camera

                   http://wiki.ros.org/image_pipeline/CameraInfo

                   http://answers.ros.org/question/11361/how-to-publish-sensor_msgscamerainfo-messages/
**************************************************************************/

#include <ros/ros.h>
#include <ros/time.h>
//#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/CameraInfo.h>
#include "sensor_msgs/Image.h"
#include "stdio.h"
#include "string.h"


void ScanKinect_CameraInfo(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
    //print out some information 
    printf("camera_info:\n\theight : %d", camera_info->height);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "p3_kinect");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    ros::Time time_now;
    ros::Time time_begin = ros::Time::now();

    //subscriber
    ros::Subscriber camera_info = nh.subscribe<sensor_msgs::CameraInfo>("/camera/dept/camera_info", 100, ScanKinect_CameraInfo);

    printf("Now read Camera Info:\n");
 
    while (ros::ok())
    {
        time_now = ros::Time::now();
        printf("time : %6.2fs\n",time_now.toSec()-time_begin.toSec());
        loop_rate.sleep();
        ros::spinOnce();
    }

    //ros::spin();
    return 0;

}
