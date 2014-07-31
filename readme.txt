Important tips：

1. if you change the package name after cloning this package, you need to modify three files which are
 1) in CMakeLists.txt
   project(pioneer3_control_ros)  -->change the package name "pioneer3_control_ros" to what you rename it.
   
 2) in /cfg/RosAria.cfg

   PACKAGE = "Pioneer3_Control_ROS" -->change the package name "pioneer3_control_ros" to what you rename it.
   
 3) in package.xml
  <name>pioneer3_control_ros</name> -->change the package name "pioneer3_control_ros" to what you rename it.
  
  if you don't do this, when you want to compile these files, it won't succeed.
  
2. follow the document "pioneer3_control_ros" to understad this package.

3. It is not allowed to run two ros nodes with the same name, thus in each robot the ros node name is differnent,
   which is defined as follows
  {
      //for local computer
      ros::init(argc,argv, "RosAria_local");
      ...
  }

  thus, even the same topics published in different robots have different name, e.g.,
  in local computer, 
      topic "sonar" is named as "Ros_local/sonar"
  while in another computer with IP address 192.168.0.254, the name is
      "RosAria_p3at_4_254/sonar".
      
  so, if you can't get the message correctly, please check whether the topic you subscribe is right.

