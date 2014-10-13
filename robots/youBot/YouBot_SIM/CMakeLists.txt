cmake_minimum_required(VERSION 2.8.3)

project(youbot_sim)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  #rosbuild_init()
  #rosbuild_find_ros_package( rtt )
  #rosbuild_find_ros_package(soem)
  #rosbuild_find_ros_package(youbot_driver)
  #set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
  #rosbuild_genmsg()
#  rosbuild_include(rtt_rosnode GenerateRTTtypekit)
#  ros_generate_rtt_typekit(YouBot_OODL)

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/external/cmake_modules)

find_package(catkin REQUIRED COMPONENTS rtt_ros tf rtt_std_msgs rtt_sensor_msgs rtt_motion_control_msgs rtt_nav_msgs rtt_geometry_msgs cmake_modules youbot_driver)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
#include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# Determine target
#set(OROCOS_TARGET $ENV{OROCOS_TARGET})

#include(${CMAKE_CURRENT_SOURCE_DIR}/../eclipse.cmake)

#V-REP include libraries 
SET(VREP_PATH "../../../simulator/V-REP/programming/remoteApi;../../../simulator/V-REP/programming/common;" CACHE STRING "My include dir you may add additional search paths here. Use ; to separate multiple paths.")
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
INCLUDE_DIRECTORIES(../../../simulator/V-REP/programming/remoteApi ../../../simulator/V-REP/programming/include)

add_library(RemoteApi SHARED ../../../simulator/V-REP/programming/remoteApi/extApi.c ../../../simulator/V-REP/programming/remoteApi/extApiPlatform.c)

FIND_PACKAGE(OODL_YouBot)
IF (OODL_YOUBOT_FOUND)
	MESSAGE(STATUS "SUCCESSFUL: OODL_YouBot found in ${OODL_YOUBOT_CONFIG_DIR}")
	ADD_DEFINITIONS(-DOODL_YOUBOT_CONFIG_DIR="${OODL_YOUBOT_CONFIG_DIR}")
ELSE (OODL_YOUBOT_FOUND) 
	MESSAGE(STATUS "ERROR: OODL_YouBot not found")
ENDIF (OODL_YOUBOT_FOUND)

#
# Compile stuff
#


SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
MESSAGE(STATUS "LIB PATH ${LIBRARY_OUTPUT_PATH}")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

ADD_DEFINITIONS(-DETHERCAT_MASTER_WITHOUT_THREAD)
ADD_DEFINITIONS(-DCFG_YOUBOT_BASE="${OODL_YOUBOT_INCLUDE_DIR}/config")
ADD_DEFINITIONS(-DSIM_ADDRESS="192.168.1.100")
ADD_DEFINITIONS(-DSIM_PORT=10001)

#defines the core sources and includes
#SET(COMMON_SRC_DIR ${CMAKE_HOME_DIRECTORY}/src)
SET(COMMON_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${USE_OROCOS_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include")

#SET(CMAKE_BUILD_TYPE Debug)

#include all CMakeList.txt files within this package
#each generates one library or application

ADD_SUBDIRECTORY(src)

#ADD_SUBDIRECTORY(examples) 

orocos_generate_package(INCLUDE_DIRS include DEPENDS rtt_ros)

endif()
