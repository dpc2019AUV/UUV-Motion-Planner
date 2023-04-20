cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

#add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -march=native -O3 -Wall")
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -Wall -g -fopenmp" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  map
  geometry_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES path_planning
CATKIN_DEPENDS roscpp std_msgs map
#  DEPENDS system_lib
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

add_library( path_planning
  src/path_planning.cpp
  src/dubins.cpp 
)

target_link_libraries( path_planning 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)








