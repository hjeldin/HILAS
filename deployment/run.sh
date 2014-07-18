#!/bin/bash

export YOUR_WORKSPACE="/home/altair/workspace/orocos"

source /opt/ros/fuerte/setup.bash
source /opt/ros/fuerte/stacks/orocos_toolchain/env.sh
source $YOUR_WORKSPACE/setup.bash

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$YOUR_WORKSPACE

#export ROS_MASTER_URI=http://youbot-01:11311
export ROS_MASTER_URI=http://altair-XPS-8500:11311
export YOUBOT_PATH=$(rosstack find youbot-stack)

export LUA_PATH="$LUA_PATH;`rospack find ocl`/lua/modules/?.lua"
export LUA_PATH="$LUA_PATH;`rospack find kdl`/?.lua"
export LUA_PATH="$LUA_PATH;`rospack find rFSM`/?.lua"
export LUA_PATH="$LUA_PATH;`rospack find rttlua_completion`/?.lua"
#export LUA_PATH="$LUA_PATH;`rospack find youbot_master_rtt`/lua/?.lua"
export LUA_PATH="$LUA_PATH;`rospack find kdl_lua`/lua/?.lua"
export LUA_PATH="$LUA_PATH;`rosstack find youbot-stack`/deployment/?.lua"
export LUA_CPATH="$LUA_CPATH;`rospack find rttlua_completion`/?.so"
export PATH="$PATH:`rosstack find orocos_toolchain`/install/bin"

#export RTT_MOTION_CONTROL_MSGS_PATH=$(rospack find rtt_motion_control_msgs)/lib
#export YOUBOT_ADAPTERS=$(rospack find YouBot_adapters)/lib/orocos
#export TSIM_ADAPTERS=$(rospack find TSimAdapters)/lib/orocos
#export YOUBOT_OODL_PATH=$(rospack find YouBot_OODL)/lib/orocos
#export YOUBOT_MASTER=$(rospack find Master_executive)/lib/orocos
#export RTT_BRICS_ACTUATOR=$(rospack find rtt_brics_actuator)/lib
#export RAW_ARM_BRIDGE=$(rospack find raw_arm_bridge_ros_orocos)/lib/orocos
#export RTT_PATH=$(rospack find rtt)/../install
#export OCL_PATH=$(rospack find ocl)/../install/lib/orocos/gnulinux
#export DATATRACER_PATH=$(rospack find DataTracer)/lib/orocos

#export RTT_COMPONENT_PATH=${RTT_PATH}:${OCL_PATH}:${RTT_MOTION_CONTROL_MSGS_PATH}:${YOUBOT_OODL_PATH}:${YOUBOT_ADAPTERS}:${TSIM_ADAPTERS}:${RAW_ARM_BRIDGE}:${RTT_BRICS_ACTUATOR}:${DATATRACER_PATH}:${YOUBOT_MASTER}

#echo ${RTT_COMPONENT_PATH}

#gdb deployer-gnulinux

#gdb rttlua-gnulinux
rosrun ocl rttlua-gnulinux -i $1
#rosrun ocl deployer-gnulinux -s $1
