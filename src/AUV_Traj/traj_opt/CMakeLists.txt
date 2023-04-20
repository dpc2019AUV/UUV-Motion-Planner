cmake_minimum_required(VERSION 3.0.2)
project(traj_opt)

#add_compile_options(-std=c++11)
#SET(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -pthread ")
#-fopenmp

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  map
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
#find_package(NLopt REQUIRED)
find_package(OpenBLAS REQUIRED)
message(WARNING "OpenBLAS_VERSION: ${OpenBLAS_VERSION}")

add_message_files(
  FILES
  TrajInfo.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenBLAS_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

catkin_package(
INCLUDE_DIRS include
LIBRARIES traj_opt
CATKIN_DEPENDS roscpp std_msgs map message_runtime
#  DEPENDS system_lib
)

## Declare a C++ library
add_library(traj_opt
  src/traj_opt.cpp
)

target_link_libraries(traj_opt 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenBLAS_LIBRARIES}
  -lpthread
  #-L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl  
  )


