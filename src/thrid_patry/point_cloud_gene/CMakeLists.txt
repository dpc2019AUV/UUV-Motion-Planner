cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_gene)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud_gene
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(global_point_cloud src/point_cloud_gene.cpp)
target_link_libraries(global_point_cloud
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)



