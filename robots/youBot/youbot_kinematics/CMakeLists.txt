cmake_minimum_required(VERSION 2.8.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(youbot_kinematics)
find_package(catkin REQUIRED COMPONENTS kdl_parser rtt_ros tf_conversions rtt_ros rtt_std_msgs rtt_sensor_msgs rtt_motion_control_msgs rtt_nav_msgs rtt_geometry_msgs)
find_package(orocos_kdl REQUIRED)
find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-marshalling)
# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${USE_OROCOS_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include")

ADD_DEFINITIONS(-DURDF_PATH_TO_FILE="${PROJECT_SOURCE_DIR}/../urdf_model/youbot.urdf")

# Rest of the configuration is in src/
add_subdirectory(src)

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package(INCLUDE_DIRS include DEPENDS)
