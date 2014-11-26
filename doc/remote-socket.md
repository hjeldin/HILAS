HILAS remote commander
===

When starting the deployer with usageMode=REMOTE, the deployer will spawn a socket server which will listen for commands and returns data accordingly.  
Commands sent to the deployer through socket server are whitelisted; this means that only some commands are exposed to the network.  

A list of these commands can be found in _HILAS_HOME/deployer/include/definitions.lua_ in the hash_remote_command.  
To extend these commands, simply add a new line composed by `["MY_REMOTE_COMMAND"] = "do_something_in_deployer"`

Command list
---
* HW_ARM_MODE_POS
* HW_ARM_MODE_VEL
* HW_ARM_MODE_TOR
* HW_ARM_MODE_STOP
* SIM_ARM_MODE_POS
* SIM_ARM_MODE_VEL
* SIM_ARM_MODE_TOR
* SIM_ARM_MODE_STOP
* HW_BASE_MODE_POS
* HW_BASE_MODE_VEL
* HW_BASE_MODE_TOR
* HW_BASE_MODE_STOP
* HW_BASE_MODE_TWIST
* SIM_BASE_MODE_POS
* SIM_BASE_MODE_VEL
* SIM_BASE_MODE_TOR
* SIM_BASE_MODE_STOP
* SIM_BASE_MODE_TWIST
* CARTESIAN_START
* CARTESIAN_STOP
* DEPLOYER_EXIT
* SWITCH_TO_HW
* SWITCH_TO_SIM
* SIM_BLOCK_YOUBOT_POSITION
* HW_BLOCK_YOUBOT_POSITION