#!/bin/bash

source /opt/ros/fuerte/setup.bash
source /opt/ros/fuerte/stacks/orocos_toolchain/env.sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/k12/youbot

#export ROS_MASTER_URI=http://youbot-dev:11311
export ROS_MASTER_URI=http://localhost:11311

export YOUBOT_PATH=$(rosstack find youbot-stack)

RTTLUA_MODULES=`rospack find ocl`/lua/modules/?.lua
if [ "x$LUA_PATH" == "x" ]; then
    LUA_PATH=";;"
fi
export LUA_PATH="$LUA_PATH;$RTTLUA_MODULES;$YOUBOT_PATH/external/rFSM/?.lua"

echo $LUA_PATH

export RTT_MOTION_CONTROL_MSGS_PATH=$(rospack find rtt_motion_control_msgs)/lib
export YOUBOT_ADAPTERS=$(rospack find YouBot_adapters)/lib/orocos
export TSIM_ADAPTERS=$(rospack find TSimAdapters)/lib/orocos
export YOUBOT_OODL_PATH=$(rospack find YouBot_OODL)/lib/orocos
export YOUBOT_MASTER=$(rospack find Master_executive)/lib/orocos
#export RTT_BRICS_ACTUATOR=$(rospack find rtt_brics_actuator)/lib
#export RAW_ARM_BRIDGE=$(rospack find raw_arm_bridge_ros_orocos)/lib/orocos
export RTT_PATH=$(rospack find rtt)/../install
export OCL_PATH=$(rospack find ocl)/../install/lib/orocos/gnulinux
#export DATATRACER_PATH=$(rospack find DataTracer)/lib/orocos

export RTT_COMPONENT_PATH=${RTT_PATH}:${OCL_PATH}:${RTT_MOTION_CONTROL_MSGS_PATH}:${YOUBOT_OODL_PATH}:${YOUBOT_ADAPTERS}:${TSIM_ADAPTERS}:${RAW_ARM_BRIDGE}:${RTT_BRICS_ACTUATOR}:${DATATRACER_PATH}:${YOUBOT_MASTER}

echo ${RTT_COMPONENT_PATH}

rosrun ocl deployer-gnulinux -s $1 $2 $3 $4 $5
