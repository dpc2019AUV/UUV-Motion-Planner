cmake_minimum_required(VERSION 3.0.2)
project(local_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  traj_msg
  #message_generation
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
#add_subdirectory(traj_msg)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES local_map
  CATKIN_DEPENDS roscpp rospy std_msgs traj_msg sensor_msgs
#  DEPENDS system_lib
)

# add_message_files(
#   FILES
#   TrajCmd.msg
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  traj_msg
)

add_executable(local_map src/local_map.cpp)
target_link_libraries(local_map
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)
#add_dependencies(local_map ${catkin_EXPORTED_TARGETS})
#add_dependencies(local_map ${PROJECT_NAME}_generate_messages_cpp)
