cmake_minimum_required(VERSION 3.0.2)
project(planner)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_COMPILER "g++")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -g -pthread " )
# -fopenmp
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  map
  path_planning
  traj_opt
  #message_generation
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# add_message_files(
#   FILES
#   TrajCmd.msg
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

include_directories(
  #SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner
  CATKIN_DEPENDS roscpp std_msgs map path_planning traj_opt
#  DEPENDS system_lib
)

add_executable(traj_planner src/planner_node.cpp 
                            src/plan_manage.cpp )

target_link_libraries(traj_planner
    ${catkin_LIBRARIES}
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
#add_dependencies(traj_server ${catkin_EXPORTED_TARGETS})

