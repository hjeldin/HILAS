cmake_minimum_required(VERSION 2.8.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(youbot_cmddemux)
find_package(catkin REQUIRED COMPONENTS rtt_ros rtt_motion_control_msgs)
# Defines the orocos_* cmake macros. See that file for additional
# documentation.

include("$ENV{HILAS_HOME}/hilas/definitions/Definition.cmake")

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${USE_OROCOS_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories(${HILAS_COMMON})
include_directories(${HILAS_ROBOTS}/youBot)

# Rest of the configuration is in src/
add_subdirectory(src)

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package(INCLUDE_DIRS include DEPENDS)