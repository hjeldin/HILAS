#/bin/bash

roslaunch robot_state_publisher robot_state_publisher.launch &

rttlua-gnulinux -i deployer_oodl.lua