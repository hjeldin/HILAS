#!/bin/bash

source /opt/ros/electric/setup.bash;
source /opt/ros/electric/stacks/orocos_toolchain/env.sh;

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/youbot/ros_stacks
export ROS_MASTER_URI=http://wallace:11311
#export ROS_HOSTNAME=10.0.3.190
source ./start_roscore.sh
export YOUBOT_PATH=$(rosstack find youbot-stack)

RTTLUA_MODULES=`rospack find ocl`/lua/modules/?.lua
if [ "x$LUA_PATH" == "x" ]; then
    LUA_PATH=";;"
fi
export LUA_PATH="$LUA_PATH;$RTTLUA_MODULES;$YOUBOT_PATH/external/rFSM/?.lua"

echo $LUA_PATH

export RTT_MOTION_CONTROL_MSGS_PATH=$(rospack find rtt_motion_control_msgs)/lib
export YOUBOT_ADAPTERS=$(rospack find YouBot_adapters)/lib/orocos
export YOUBOT_OODL_PATH=$(rospack find YouBot_OODL)/lib/orocos
export RTT_PATH=$(rospack find rtt)/../install
export OCL_PATH=$(rospack find ocl)/../install
export JOYSTICK_PATH=$(rospack find YouBot_joystick)/lib/orocos

export RTT_COMPONENT_PATH=${RTT_PATH}:${OCL_PATH}:${RTT_MOTION_CONTROL_MSGS_PATH}:${YOUBOT_OODL_PATH}:${YOUBOT_ADAPTERS}:${JOYSTICK_PATH}

export ORO_LOGLEVEL=6
export BASE_LOG_LEVEL="DEBUG"

gdb --args ${OCL_PATH}/bin/deployer-gnulinux -ldebug -s $1 -- $2 $3 $4 $5

