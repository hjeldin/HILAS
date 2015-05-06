cmake_minimum_required(VERSION 2.8.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(youbot_kinematics)
find_package(catkin REQUIRED COMPONENTS kdl_parser rtt_ros tf_conversions rtt_ros rtt_std_msgs rtt_sensor_msgs rtt_motion_control_msgs rtt_nav_msgs rtt_geometry_msgs youbot_driver)
find_package(orocos_kdl REQUIRED)
find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-marshalling)
# Defines the orocos_* cmake macros. See that file for additional
# documentation.

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/external/cmake_modules)

FIND_PACKAGE(OODL_YouBot)
IF (OODL_YOUBOT_FOUND)
	MESSAGE(STATUS "SUCCESSFUL: OODL_YouBot found in ${OODL_YOUBOT_CONFIG_DIR}")
	ADD_DEFINITIONS(-DOODL_YOUBOT_CONFIG_DIR="${OODL_YOUBOT_CONFIG_DIR}")
ELSE (OODL_YOUBOT_FOUND) 
	MESSAGE(STATUS "ERROR: OODL_YouBot not found")
ENDIF (OODL_YOUBOT_FOUND)

include("$ENV{HILAS_HOME}/hilas/definitions/Definition.cmake")

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${USE_OROCOS_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES(${HILAS_COMMON})
INCLUDE_DIRECTORIES(${HILAS_DEFINITIONS})
INCLUDE_DIRECTORIES(${HILAS_ROBOTS}/youbot)
INCLUDE_DIRECTORIES(${OODL_YOUBOT_INCLUDE_DIR})

ADD_DEFINITIONS(-DURDF_PATH_TO_FILE="${PROJECT_SOURCE_DIR}/../youbot_description/robots/youbot.urdf")
ADD_DEFINITIONS(-DCFG_YOUBOT_BASE="${OODL_YOUBOT_CONFIG_DIR}/")

# Rest of the configuration is in src/
add_subdirectory(src)

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package(INCLUDE_DIRS include DEPENDS)
