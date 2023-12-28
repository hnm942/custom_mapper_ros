cmake_minimum_required(VERSION 2.8.3)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

project(custom_mapper_ros)

set(CMAKE_VERBOSE_MAKEFILE FALSE)

add_definitions("-std=c++14")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# find catkin dependencies
set(REQ_CATKIN_PKGS   
  roscpp
  nodelet
  fla_utils
  pcl_ros
  pcl_msgs
  pcl_conversions
  tf2_ros
  nav_msgs
  visualization_msgs
  tf2_sensor_msgs
  message_generation
  image_transport
  cv_bridge
  fla_msgs
  snapstack_msgs
  )
find_package(catkin REQUIRED COMPONENTS ${REQ_CATKIN_PKGS})

# find system dependencies
find_package(PCL 1.7 REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(global_mapper REQUIRED)

# setup downstream dependencies
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${REQ_CATKIN_PKGS}
  DEPENDS PCL
  )

# setup global include directories
include_directories(
  include
  ${global_mapper_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )



add_library(custom_mapper 
  src/custom_mapper/octree_grid.cpp)
target_link_libraries(custom_mapper
  ${catkin_LIBRARIES}
  )

add_library(custom_mapper_ros
  src/custom_mapper_ros/custom_mapper_ros.cpp
  )

target_link_libraries(custom_mapper_ros
  ${catkin_LIBRARIES}
  )

add_executable(custom_mapper_node 
  src/custom_mapper_node.cpp)

target_link_libraries(custom_mapper_node
  ${catkin_LIBRARIES}
  custom_mapper_ros
  )