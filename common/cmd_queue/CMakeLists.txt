cmake_minimum_required(VERSION 2.8.3)

project(cmd_queue)

find_package(catkin REQUIRED COMPONENTS rtt_ros rtt_geometry_msgs rtt_motion_control_msgs roscpp kdl_parser orocos_kdl tf_conversions cmake_modules)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${USE_OROCOS_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include")

add_subdirectory(src)

orocos_generate_package(INCLUDE_DIRS include DEPENDS) #rtt_geometry_msgs rtt_ros)