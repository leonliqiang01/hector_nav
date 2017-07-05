cmake_minimum_required(VERSION 2.8.3)
project(hector_nav2)


IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O2 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  pcl_ros
  roscpp
  rospy
  std_msgs
  genmsg
  message_filters
  dynamic_reconfigure
  message_generation
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  image_geometry 
  image_transport
  visualization_msgs
  octomap_ros
  octomap_msgs
  pcl_conversions
  sensor_msgs
)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
	cfg/param_reconfig.cfg
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
)

add_message_files(
  FILES
  ObstacleMsg.msg
  TrajectoryPointMsg.msg
  TrajectoryMsg.msg
  FeedbackMsg.msg
)
generate_messages(
   DEPENDENCIES
   geometry_msgs 
   std_msgs
)
catkin_package(
# INCLUDE_DIRS include
# LIBRARIES hector_nav
# CATKIN_DEPENDS roscpp rospy std_msgs
# DEPENDS system_lib
CATKIN_DEPENDS
	message_runtime

)

include_directories(
#   Guidance/include
  include/waypoint_type 
  include/hector_nav 
  include/position_control 
  include/waypoint_handle
  include/trajectory_generate
  include/octomap_construct
  include/teb_planner
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${G2O_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS}
		  #${G2O_LIBRARY_DIRS}
		  ${OCTOMAP_LIBRARY_DIRS}
		  ${OpenCV_LIBRARY_DIRS}
		  ${EIGEN3_LIBRARY_DIRS}
		  
		 )
# add_subdirectory(Guidance)
## Declare a C++ executable
add_executable(hector_nav2 src/hector_nav.cpp src/position_control.cpp src/trajectory_generate.cpp src/octomap_construct.cpp
	src/teb_planner.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(hector_nav2 hector_nav2_generate_messages_cpp)
add_dependencies(hector_nav2 hector_nav2_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(hector_nav2
  ${catkin_LIBRARIES}
  ${G2O_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)