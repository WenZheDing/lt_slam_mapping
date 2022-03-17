cmake_minimum_required(VERSION 2.8.3)
project(calibration)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
)

find_package( OpenCV 3.1 REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS  roscpp rospy std_msgs sensor_msgs tf
  DEPENDS EIGEN3 PCL 
)

add_executable(pose_estimation_3d3d
src/pose_estimation_3d3d.cpp
)

target_link_libraries(pose_estimation_3d3d
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
 ${OpenCV_LIBS})