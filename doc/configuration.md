HILAS configuration file
===

A default configuration file could be located in HILAS_HOME/config/hilas.ini.


	[hilas]
	runMode=SIM
	deployerMode=LUA_DEPLOYER
	usageMode=DEBUG
	useROS=true
	socketaddr=*
	socketport=22223

	[robot]
	name=youBot

	[simulator]
	ip=127.0.0.1
	port=10001

	[ROS]
	topicArmPosCommand=/arm_1/arm_controller/position_command
	topicArmVelCommand=/arm_1/arm_controller/velocity_command
	topicArmTorCommand=/arm_1/arm_controller/torque_command
	topicBaseTwistCommand=/cmd_vel
	topicBaseOdomState=/odom
	topicGripperPosCommand=/arm_1/gripper_controller/position_command
	topicSimArmJointState=/vrep/hw_rx/arm_1/joint_state
	topicSimBaseJointState=/vrep/hw_rx/base/joint_state
	topicSimOdomState=/vrep/hw_rx/odom

`[hilas]`
---

**runMode**: usage of the architecture.  
*Possible values:*
* SIM (only simulation)
* HW (only hardware)
* BOTH (simulation AND hardware)

**deployerMode**: deployer mode.  
*Possible values:*
* LUA_DEPLOYER
* OPS_DEPLOYER

**usageMode**:   
*Possible values:* 
* REMOTE - starts a socket server waiting commands. Check [it's documentation](remote-socket.html).
* LOCAL - do not use
* DEBUG - default value, accepts commands from lua console

**useROS**:
*Possible values:*
* true - will need a ROS core enabled 
* false - will not need a ROS core

**socketaddr** and **socketport**:
Specifies the address/port the server will listen if started with `usageMode=REMOTE`


`[robot]`:
---
**name**: name of the ini file and the folders in which the deployer will find it's component

`[simulator]`:
---
**ip** and **port**: address/port of the machine running v-rep

`[ROS]`
---
**TBD**