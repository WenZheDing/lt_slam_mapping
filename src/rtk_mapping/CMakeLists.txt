  cmake_minimum_required(VERSION 3.0.2)
  project(rtk_mapping)

  ## Compile as C++11, supported in ROS Kinetic and newer
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    location_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    ndt_cpu
    # ndt_omp
    tf
    cv_bridge
    message_generation
  )

  find_package(Eigen3)
  find_package(PCL 1.8 REQUIRED)
  find_package(GTSAM REQUIRED QUIET)    
  find_package(Boost REQUIRED COMPONENTS thread system timer chrono)
  find_package(OpenCV REQUIRED QUIET)

  add_message_files(
   FILES
   node.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
  )

  message("Boost_DIRS=${Boost_INCLUDE_DIRS}")
  message("Boost_LIBRARIES_DIRS=${Boost_LIBRARIES}")
  message("PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if your package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES rtk_mapping
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  #  DEPENDS system_lib
  )

  ###########
  ## Build ##
  ###########

  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  include_directories(  
    include
    include/functions
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}  
  )

  link_directories(
    include
    ${OpenCV_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
  )

  add_executable(map src/map.cpp src/functions/func.cpp src/Scancontext.cpp )
  target_link_libraries(map ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} gtsam ${OpenCV_LIBRARIES})
  add_dependencies(map ${catkin_EXPORTED_TARGETS})
  
  add_executable(pcd_viewer src/pcd_viewer.cpp)
  target_link_libraries(pcd_viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

  add_executable(imu src/imu.cpp)
  target_link_libraries(imu ${catkin_LIBRARIES})

  add_executable(gtsam_test src/gtsam_test.cpp)
  target_link_libraries(gtsam_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} gtsam)
  add_dependencies(gtsam_test ${catkin_EXPORTED_TARGETS})

