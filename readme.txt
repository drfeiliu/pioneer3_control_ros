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
