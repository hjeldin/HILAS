cmake_minimum_required(VERSION 2.8.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(simulator)
find_package(catkin REQUIRED COMPONENTS)

include("$ENV{HILAS_HOME}/hilas/definitions/Definition.cmake")

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

#V-REP include libraries 
SET(VREP_PATH "V-REP/programming/remoteApi;V-REP/programming/common;" CACHE STRING "My include dir you may add additional search paths here. Use ; to separate multiple paths.")
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)

INCLUDE_DIRECTORIES(V-REP/programming/remoteApi V-REP/programming/include)
add_library(RemoteApi SHARED V-REP/programming/remoteApi/extApi.c V-REP/programming/remoteApi/extApiPlatform.c)