#!/bin/bash

RT_CART_CONTROLLER_LUA="youbot_cartesian_controller.launch"
RT_CART_CONTROLLER_DEPLOYER="youbot_cartesian_controller_deployer.launch"

function usage() {
    cat <<EOF
youbot rt_cartesian controller
usage: $1 <option>
  options:  -1:  run cartesian controller test (LUA deployment script)
	    -2:  run cartesian controller test (Orocos deployment script) NOTE: rosrun ocl deployer-gnulinux -s deploy.ops (new terminal) 
            -h: print this
EOF
}

if [[ -z $1 || $1 == "-h" ]]; then
    usage
elif [[ $1 == "-1" ]]; then
    roslaunch "launch/$RT_CART_CONTROLLER_LUA"
elif [[ $1 == "-2" ]]; then
    roslaunch "launch/$RT_CART_CONTROLLER_DEPLOYER"
else
    usage
fi
