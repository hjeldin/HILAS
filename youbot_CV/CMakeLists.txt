cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if(NOT OpenCV_DIR)
 find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV" HINTS    "/opt/ros/fuerte/share/OpenCV/")
endif(NOT OpenCV_DIR)
FIND_PACKAGE(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/../../../V-REP_PRO_EDU_V3_0_5c_64_Linux/programming)

#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(${PROJECT_SOURCE_DIR}/../../vrep/vrep_common/msg_gen/cpp/include)

rosbuild_add_executable(pointcloud_save src/pointcloud_save.cpp src/v_repLib.cpp)
rosbuild_add_executable(camera_calibrate src/camera_calibrate.cpp)
rosbuild_add_executable(pointcloud_triangulate src/pointcloud_triangulate.cpp)
rosbuild_add_executable(pointcloud_streamer src/pointcloud_streamer.cpp)


target_link_libraries(camera_calibrate ${OpenCV_LIBS})