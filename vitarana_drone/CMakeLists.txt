cmake_minimum_required(VERSION 3.0.2)
project(vitarana_drone)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy  
  std_srvs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs
  tf 
  tf2_ros
  std_msgs
  gazebo_ros
)
include_directories( include ${catkin_INCLUDE_DIRS})


find_package(gazebo REQUIRED)
include_directories( include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   prop_speed.msg
   edrone_cmd.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Gripper.srv

)

 generate_messages(
   DEPENDENCIES
   std_msgs  
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/SensorModel.cfg
  cfg/GNSS.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
	DEPENDS 
	    roscpp 
	    gazebo_ros
      std_msgs 
      geometry_msgs 
      tf
      message_runtime
      dynamic_reconfigure

)


###########
## Build ##
###########

add_library(gazebo_edrone_propulsion src/gazebo_edrone_propulsion.cpp)
add_dependencies(gazebo_edrone_propulsion vitarana_drone_generate_messages_cpp)
target_link_libraries (gazebo_edrone_propulsion ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_gps src/gazebo_ros_gps.cpp)
add_dependencies(gazebo_ros_gps vitarana_drone_generate_messages_cpp)
target_link_libraries (gazebo_ros_gps ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  gazebo_edrone_propulsion
  gazebo_ros_gps
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
