Important tips：

1. if you change the package name after cloning this package, you need to modify three files which are
 1) in CMakeLists.txt
   project(Pioneer3_Control_ROS)  -->change the package name "Pioneer3_Control_ROS" to what you rename it.
   
 2) in /cfg/RosAria.cfg

   PACKAGE = "Pioneer3_Control_ROS" -->change the package name "Pioneer3_Control_ROS" to what you rename it.
   
 3) in package.xml
  <name>Pioneer3_Control_ROS</name> -->change the package name "Pioneer3_Control_ROS" to what you rename it.
  
  if you don't do this, when you want to compile these files, it won't succeed.
  
2. follow the document "Pioneer3_Cotrol_ROS" to understad this package.
