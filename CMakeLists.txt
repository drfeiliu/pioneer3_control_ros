# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pioneer3_control_ros)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation roscpp nav_msgs geometry_msgs sensor_msgs tf
  dynamic_reconfigure)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
  FILES
  BumperState.msg
)
#uncomment if you have defined services
#add_service_files(
#  FILES
#  # TODO: List your msg files here
#)

generate_dynamic_reconfigure_options(cfg/RosAria.cfg)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

# The installation package provided by Adept doesn't follow Debian policies
if(EXISTS "/usr/local/Aria/include/Aria.h")
  add_definitions(-DADEPT_PKG)
  include_directories( /usr/local/Aria/include)
  link_directories(/usr/local/Aria/lib)
endif()

add_executable(RosAria RosAria.cpp)
add_dependencies(RosAria rosaria_gencfg)
add_dependencies(RosAria rosaria_gencpp)

target_link_libraries(RosAria ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)
set_target_properties(RosAria PROPERTIES COMPILE_FLAGS "-fPIC")


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs tf
)
add_executable(test_cmd test_cmd.cpp)
add_dependencies(test_cmd rosaria_gencfg)
add_dependencies(test_cmd rosaria_gencpp)
target_link_libraries(test_cmd ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)


##16,May,2014
add_executable(rob_key rob_key.cpp)
add_dependencies(rob_key rosaria_gencfg)
add_dependencies(rob_key rosaria_gencpp)
target_link_libraries(rob_key ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)

add_executable(android_teleop android_teleop.cpp)
add_dependencies(android_teleop rosaria_gencfg)
add_dependencies(android_teleop rosaria_gencpp)
target_link_libraries(android_teleop ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)

add_executable(test_record_data test_record_data.cpp)
add_dependencies(test_record_data rosaria_gencfg)
add_dependencies(test_record_data rosaria_gencpp)
target_link_libraries(test_record_data ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)

add_executable(robotWander robotWander.cpp)
add_dependencies(robotWander rosaria_gencfg)
add_dependencies(robotWander rosaria_gencpp)
target_link_libraries(robotWander ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)

add_executable(robotDemo robotDemo.cpp)
add_dependencies(robotDemo rosaria_gencfg)
add_dependencies(robotDemo rosaria_gencpp)
target_link_libraries(robotDemo ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)

add_executable(p3_sonar p3_sonar.cpp)
add_dependencies(p3_sonar rosaria_gencfg)
add_dependencies(p3_sonar rosaria_gencpp)
target_link_libraries(p3_sonar ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)

add_executable(p3_kinect p3_kinect.cpp)
add_dependencies(p3_kinect rosaria_gencfg)
add_dependencies(p3_kinect rosaria_gencpp)
target_link_libraries(p3_kinect ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)

add_executable(p3_sonar_following_moving_object p3_sonar_following_moving_object.cpp)
add_dependencies(p3_sonar_following_moving_object rosaria_gencfg)
add_dependencies(p3_sonar_following_moving_object rosaria_gencpp)
target_link_libraries(p3_sonar_following_moving_object ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)

