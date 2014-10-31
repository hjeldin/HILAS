-- Enum definition
SIM, HW, BOTH, LUA_DEPLOYER, OPS_DEPLOYER, VREP, OODL, REMOTE, LOCAL = 0, 1, 2, 3, 4, 5 ,6, 7, 8

-- Definition of an hash table for configuration file naming
hash_config = {
["SIM"] = SIM,
["HW"] = HW,
["BOTH"] = BOTH,
["LUA_DEPLOYER"] = LUA_DEPLOYER,
["OPS_DEPLOYER"] = OPS_DEPLOYER,
["VREP"] = VREP,
["OODL"] = OODL,
["REMOTE"] = REMOTE,
["LOCAL"] = LOCAL
}

-- Definition of an hash table for mark remote callable functions
hash_remote_command = {
["HW_ARM_MODE_POS"] = "for i=1,robot_arm_count do armSetCtrlModes(OODL,i,1) end",
["HW_ARM_MODE_VEL"] = "for i=1,robot_arm_count do armSetCtrlModes(OODL,i,2) end",
["HW_ARM_MODE_TOR"] = "for i=1,robot_arm_count do armSetCtrlModes(OODL,i,3) end",
["HW_ARM_MODE_STOP"] = "for i=1,robot_arm_count do armSetCtrlModes(OODL,i,4) end",
["SIM_ARM_MODE_POS"] = "for i=1,robot_arm_count do armSetCtrlModes(SIM,i,1) end",
["SIM_ARM_MODE_VEL"] = "for i=1,robot_arm_count do armSetCtrlModes(SIM,i,2) end",
["SIM_ARM_MODE_TOR"] = "for i=1,robot_arm_count do armSetCtrlModes(SIM,i,3) end",
["SIM_ARM_MODE_STOP"] = "for i=1,robot_arm_count do armSetCtrlModes(SIM,i,4) end",
["HW_BASE_MODE_POS"] = "baseSetCtrlModes(OODL,1) end",
["HW_BASE_MODE_VEL"] = "baseSetCtrlModes(OODL,2) end",
["HW_BASE_MODE_TOR"] = "baseSetCtrlModes(OODL,3) end",
["HW_BASE_MODE_STOP"] = "baseSetCtrlModes(OODL,4) end",
["HW_BASE_MODE_TWIST"] = "baseSetCtrlModes(OODL,5) end",
["SIM_BASE_MODE_POS"] = "baseSetCtrlModes(SIM,1) end",
["SIM_BASE_MODE_VEL"] = "baseSetCtrlModes(SIM,2) end",
["SIM_BASE_MODE_TOR"] = "baseSetCtrlModes(SIM,3) end",
["SIM_BASE_MODE_STOP"] = "baseSetCtrlModes(SIM,4) end",
["SIM_BASE_MODE_TWIST"] = "baseSetCtrlModes(SIM,5) end",
["CARTESIAN_START"] = "robot_ctrl_cartesian:configure(); robot_kine:start(); robot_ctrl_cartesian:start()",
["CARTESIAN_STOP"] = "cartesian_controller_stop()",
["DEPLOYER_EXIT"] = "os.exit()",
["SWITCH_TO_HW"] = "switch_to(HW)",
["SWITCH_TO_SIM"] = "switch_to(SIM)",
["SIM_BLOCK_YOUBOT_POSITION"] = "block_youbot_position(SIM)",
["HW_BLOCK_YOUBOT_POSITION"] = "block_youbot_position(OODL)",
}

-- Weird variables
rttlib.color = true
running = true

-- HILAS SETUP --
configTable = inifile.parse('../config/hilas.ini')

robot_name = configTable['robot']['name']
robot_arm_count = configTable['robot']['armCount']
robot_arm_joint_count = {}

for i = 1,robot_arm_count do
	robot_arm_joint_count[i] = configTable['robot']['arm'..i..'JointCount']
end

robot_base_count = configTable['robot']['baseCount']
robot_base_joint_count = {}

for i = 1,robot_base_count do
	robot_base_joint_count[i] = configTable['robot']['base'..i..'JointCount']
end

run_status = hash_config[configTable['hilas']['runMode']] --SIM
deployer_type = hash_config[configTable['hilas']['deployerMode']] --LUA_DEPLOYER
communication_type = hash_config[configTable['hilas']['usageMode']] --DEBUG
is_ros_enabled = configTable['hilas']['useROS']
socket_address = configTable['hilas']['socketaddr']
socket_port = configTable['hilas']['socketport']

CARTESIAN_GAIN_SIM = configTable['cartesian_controller']['gainHw']
CARTESIAN_GAIN_OODL = configTable['cartesian_controller']['gainSim']

kinematic_joint_nr = configTable['kinematic']['jointNr']

TOPIC_ARM_POSITION_COMMAND = configTable['ROS']['topicArmPosCommand']
TOPIC_ARM_VELOCITY_COMMAND = configTable['ROS']['topicArmVelCommand']
TOPIC_ARM_EFFORT_COMMAND = configTable['ROS']['topicArmTorCommand']
TOPIC_BASE_TWIST_COMMAND = configTable['ROS']['topicBaseTwistCommand']
TOPIC_BASE_ODOM_STATE = configTable['ROS']['topicBaseOdomState']
TOPIC_GRIPPER_POSITION_COMMAND = configTable['ROS']['topicGripperPosCommand']
SIM_TOPIC_ARM_JOINT_STATES_RX = configTable['ROS']['topicSimArmJointState']
SIM_TOPIC_BASE_JOINT_STATES_RX = configTable['ROS']['topicSimBaseJointState']
SIM_TOPIC_ODOM_STATE_RX = configTable['ROS']['topicSimOdomState']

-- GENERIC FUNCTIONS --
function firstToUpper(str)

    return (str:gsub("^%l", string.upper))

end

function safeCall(function_name)

	local status, err = pcall(function() loadstring(function_name)() end)

	if status == false then

		print("[DEPLOYER] "..err)

	end
end

-- DEPLOYER FUNCTIONS --
function ros_stream(orocos_port,topic_name)

	if is_ros_enabled == true then

		depl:stream(orocos_port,rtt.provides("ros"):topic(topic_name))

	end

end

function sim_visual_mode(a)

	if a == 1 then

		for i=1,robot_arm_count do
			depl:connect("Robot_SIM.Arm"..i..".joint_state_in","Robot_OODL.Arm"..i..".joint_state_out",cp)
		end


	if robot_base_count > 0 then

		depl:connect("Robot_SIM.Base.joint_state_in","Robot_OODL.Base.joint_state_out",cp)
		depl:connect("Robot_SIM.Base.odometry_state_in","Robot_OODL.Base.odometry_state_out",cp)
	
	end

	elseif a == 2 then

		for i=1,robot_arm_count do
			
			sim_arm_serv[i]:getPort("joint_state_in"):disconnect()

		end

	if robot_base_count > 0 then

		sim_base_serv:getPort("joint_state_in"):disconnect()
		sim_base_serv:getPort("odometry_state_in"):disconnect()

	end

	end

	sim_set_mode(a)

end

function vel_startup(stype,arm_index)

	if stype == SIM then

		armSetCtrlModes(SIM,arm_index,2)

		if robot_base_count > 0 then

			baseSetCtrlModes(SIM,5)

		end

	elseif stype == OODL then
		armSetCtrlModes(OODL,arm_index,2)

		if robot_base_count > 0 then		
	
			baseSetCtrlModes(OODL,5)
		end

	end

	cartesian_controller_start()

end

-- Definition setup functions
function simulation_setup()

	rtt.logl('Info', "Robot SIM configure.")
	robot_sim:configure()

	sim_set_mode = robot_sim:getOperation("setSimMode")

	sim_arm_serv = {}
	sim_arm_op_clear = {}
	sim_arm_op_stat = {}

	sim_grip_serv = {}

	for i = 1,robot_arm_count do
		sim_arm_serv[i] = robot_sim:provides("Arm"..i)
		sim_arm_op_clear[i] = sim_arm_serv[i]:getOperation("clearControllerTimeouts")
		sim_arm_op_stat[i] = sim_arm_serv[i]:getOperation("displayMotorStatuses")		
	end	

	for i = 1,robot_arm_count do
		sim_grip_serv[i] = robot_sim:provides("Gripper"..i)
	end

	if robot_base_count > 0 then

		sim_base_serv = robot_sim:provides("Base")
		sim_base_op_clear = sim_base_serv:getOperation("clearControllerTimeouts")
		sim_base_op_stat = sim_base_serv:getOperation("displayMotorStatuses")
	end

end

function oodl_setup()

	rtt.logl('Info', "Robot OODL configure.")
	robot_oodl:configure()

	oodl_arm_serv = {}
	oodl_arm_op_clear = {}
	oodl_arm_op_stat = {}

	oodl_grip_serv = {}

	for i = 1,robot_arm_count do
		oodl_arm_serv[i] = robot_oodl:provides("Arm"..i)
		oodl_arm_op_clear[i] = oodl_arm_serv[i]:getOperation("clearControllerTimeouts")
		oodl_arm_op_stat[i] = oodl_arm_serv[i]:getOperation("displayMotorStatuses")		
	end	

	for i = 1,robot_arm_count do
		oodl_grip_serv[i] = robot_oodl:provides("Gripper"..i)
	end

	if robot_base_count > 0 then
	
		oodl_base_serv = robot_oodl:provides("Base")
		oodl_base_op_clear = oodl_base_serv:getOperation("clearControllerTimeouts")
		oodl_base_op_stat = oodl_base_serv:getOperation("displayMotorStatuses")

	end

end

function queue_setup()

	cmd_queue:configure()
	cmd_queue:start()
	queue_op_is_loading = cmd_queue:getOperation("setIsInLoading")
	
	-- TEST --
	ros_stream("Cmd_QUEUE.ros_cartesian_command_out","/cart_debug")
end

function queue_output_disconnect()

	-- cmd_queue:getPort("arm_joint_position_command_out"):disconnect()
	-- cmd_queue:getPort("arm_joint_velocity_command_out"):disconnect()
	-- cmd_queue:getPort("arm_joint_effort_command_out"):disconnect()
	-- cmd_queue:getPort("base_cmd_twist_out"):disconnect()
	-- cmd_queue:getPort("gripper_joint_position_command_out"):disconnect()
	cmd_queue:getPort("ros_planner_command_out"):disconnect()
	cmd_queue:getPort("ros_cartesian_command_out"):disconnect()

end

function queue_input_connect()

	--ros_stream("Cmd_QUEUE.ros_arm_joint_position_command_in", TOPIC_ARM_POSITION_COMMAND))
	--ros_stream("Cmd_QUEUE.ros_arm_joint_velocity_command_in", TOPIC_ARM_VELOCITY_COMMAND))
	--ros_stream("Cmd_QUEUE.ros_arm_joint_effort_command_in", TOPIC_ARM_EFFORT_COMMAND))
	--ros_stream("Cmd_QUEUE.ros_base_cmd_twist_in", TOPIC_BASE_TWIST_COMMAND))
	--ros_stream("Cmd_QUEUE.ros_gripper_joint_position_command_in", TOPIC_GRIPPER_POSITION_COMMAND))
	ros_stream("Cmd_QUEUE.ros_planner_command_in", "/move_base_simple/goal")
	
	-- ################ No more usefull
	--depl:connect("Cmd_QUEUE.ros_cartesian_command_in","Robot_CTRL_CARTESIAN.CartesianDesiredPosition", cp)
	-- ################	

	--ros_stream("Cmd_QUEUE.ros_cartesian_command_in", "/youbot/desired_ee"))

end

function queue_input_disconnect()

	-- cmd_queue:getPort("ros_arm_joint_position_command_in"):disconnect()
	-- cmd_queue:getPort("ros_arm_joint_velocity_command_in"):disconnect()
	-- cmd_queue:getPort("ros_arm_joint_effort_command_in"):disconnect()
	-- cmd_queue:getPort("ros_base_cmd_twist_in"):disconnect()
	-- cmd_queue:getPort("ros_gripper_joint_position_command_in"):disconnect()
	cmd_queue:getPort("ros_planner_command_in"):disconnect()
	cmd_queue:getPort("ros_cartesian_command_in"):disconnect()

end

function queue_output_connect()

	-- for i = 1,robot_arm_count do

	-- 	depl:connect("Cmd_QUEUE.arm_joint_position_command_out","Robot_OODL.Arm"..i..".joint_position_command_in", cp)
	-- 	depl:connect("Cmd_QUEUE.arm_joint_velocity_command_out","Robot_OODL.Arm"..i..".joint_velocity_command_in", cp)
	-- 	depl:connect("Cmd_QUEUE.arm_joint_effort_command_out", "Robot_OODL.Arm"..i..".joint_effort_command_in", cp)
	-- 	depl:connect("Cmd_QUEUE.gripper_joint_position_command_out", "Robot_OODL.Gripper"..i..".gripper_cmd_position_in", cp)
	
	-- end

	-- if robot_base_count == 0 then

	-- 	depl:connect("Cmd_QUEUE.base_cmd_twist_out", "Robot_OODL.Base.cmd_twist_in", cp)

	-- end

	ros_stream("Cmd_QUEUE.ros_planner_command_out", "/move_base_simple/goal") -- PLANNER DA INSERIRE
	depl:connect("Cmd_QUEUE.ros_cartesian_command_out", "Robot_CTRL_CARTESIAN.CartesianDesiredPosition", cp)
	depl:connect("Cmd_QUEUE.from_cartesian_status_in", "Robot_KINE.EEPose_out", cp)
	
	--ros_stream("Cmd_QUEUE.ros_cartesian_command_out", "/cart_debug")

end

function cartesian_controller_setup()

	depl:connect("Robot_KINE.EEPose_out","Robot_CTRL_CARTESIAN.CartesianSensorPosition",cp)
	depl:connect("Robot_KINE.EETwistRTT_in","Robot_CTRL_CARTESIAN.CartesianOutputVelocity",cp)

	K = robot_ctrl_cartesian:getProperty("K")
	local gain = 0.02
	K:fromtab{gain,gain,gain,gain,gain,gain}
	ros_stream("Robot_KINE.EEPose_out","/youbot/EEPose")
end

function cartesian_controller_start()

	robot_kine:configure()
	robot_ctrl_cartesian:configure()
	robot_kine:start()
	robot_ctrl_cartesian:start()

	set_cartesian_goal(0.0,0.0,0.0)

end

function kinematic_js_weight(weights_table)

	print("Configure JointSpaceWeights")
	js_weight_port = rttlib.port_clone_conn(robot_kine:getPort("JointSpaceWeights_in"))
	js_weight = rtt.Variable("float64[]")
	js_weight:resize(kinematic_joint_nr)

	for i=1,kinematic_joint_nr do
		js_weight[i-1] = weights_table[i]
	end

	print(js_weight)
	js_weight_port:write(js_weight)

end

function cartesian_goal_connect()

	--ros_stream("Robot_CTRL_CARTESIAN.CartesianDesiredPosition","/youbot/desired_ee")
	--ros_stream("Robot_CTRL_CARTESIAN.CartesianDesiredPosition","/interactiveEEPose")

end

function cartesian_goal_disconnect()

	robot_ctrl_cartesian:getPort("CartesianDesiredPosition"):disconnect()

end

function kinematic_input_from_sim()

	depl:connect("Robot_KINE.JointState_in","Robot_SIM.Arm1.joint_state_out",cp)
	depl:connect("Robot_KINE.JointVelocities_out","Robot_SIM.Arm1.joint_velocity_command_in",cp)

	if robot_base_count > 0 then 

		depl:connect("Robot_KINE.BaseTwist_out","Robot_SIM.Base.cmd_twist_in",cp)
		depl:connect("Robot_KINE.BaseOdom_in","Robot_SIM.Base.odometry_state_out",cp)

	end

end

function kinematic_input_from_oodl()

	depl:connect("Robot_KINE.JointState_in","Robot_OODL.Arm1.joint_state_out",cp)
	depl:connect("Robot_KINE.JointVelocities_out","Robot_OODL.Arm1.joint_velocity_command_in",cp)

	if robot_base_count > 0 then

		depl:connect("Robot_KINE.BaseTwist_out","Robot_OODL.Base.cmd_twist_in",cp)
		depl:connect("Robot_KINE.BaseOdom_in","Robot_OODL.Base.odometry_state_out",cp)

	end

end

function robot_republisher_oodl()

	depl:connect("Robot_STATE_PUBLISHER.arm_state_in","Robot_OODL.Arm1.joint_state_out",cp)

	if robot_base_count > 0 then

		depl:connect("Robot_STATE_PUBLISHER.base_state_in","Robot_OODL.Base.joint_state_out",cp)
		depl:connect("Robot_STATE_PUBLISHER.odometry_state_in","Robot_OODL.Base.odometry_state_out",cp)
		ros_stream("Robot_STATE_PUBLISHER.odometry_state_out","/odom")
	end

	ros_stream("Robot_STATE_PUBLISHER.robot_state_out","/joint_states")

end

function robot_republisher_sim()

	
	depl:connect("Robot_STATE_PUBLISHER.arm_state_in","Robot_SIM.Arm1.joint_state_out",cp)

	if robot_base_count > 0 then

		depl:connect("Robot_STATE_PUBLISHER.base_state_in","Robot_SIM.Base.joint_state_out",cp)
		depl:connect("Robot_STATE_PUBLISHER.odometry_state_in","Robot_SIM.Base.odometry_state_out",cp)	
		ros_stream("Robot_STATE_PUBLISHER.odometry_state_out","/odom")

	end

	ros_stream("Robot_STATE_PUBLISHER.robot_state_out","/joint_states")

end

function disconnect_command_from_ros(stype) 

	if stype == SIM then
		arm_serv = sim_arm_serv
		base_serv = sim_base_serv
		grip_serv = sim_grip_serv
	elseif stype == OODL then
		arm_serv = oodl_arm_serv
		base_serv = oodl_base_serv		
		grip_serv = oodl_grip_serv
	end

	--ARM command  
	for i=1,robot_arm_count do

		serv[i]:getPort("joint_position_command_in"):disconnect()
		serv[i]:getPort("joint_velocity_command_in"):disconnect()
		serv[i]:getPort("joint_effort_command_in"):disconnect()
		--Gripper command
		grip_serv[i]:getPort("gripper_cmd_position_in"):disconnect()

	end

	--Base command
	if robot_base_count > 0 then

		base_serv:getPort("joint_state_in"):disconnect()
	
	end
end

function connect_command_from_ros(stype)

	--ARM command 
	for i=1,robot_arm_count do

		ros_stream("Robot_"..stype..".Arm"..i..".joint_position_command_in",TOPIC_ARM_POSITION_COMMAND)
		ros_stream("Robot_"..stype..".Arm"..i..".joint_velocity_command_in",TOPIC_ARM_VELOCITY_COMMAND)
		ros_stream("Robot_"..stype..".Arm"..i..".joint_effort_command_in",TOPIC_ARM_EFFORT_COMMAND)
		--Gripper command
		ros_stream("Robot_"..stype..".Gripper"..i..".gripper_cmd_position_in",TOPIC_GRIPPER_POSITION_COMMAND)

	end
	
	--Base command
	if robot_base_count > 0 then
	
		ros_stream("Robot_"..stype..".Base.cmd_twist_in",TOPIC_BASE_TWIST_COMMAND)
	
	end
end

function block_robot_position(mode)

	if mode == OODL then

		-- ARM --
		for i=1,robot_arm_count do

			armSetCtrlModes(OODL,i,1)

			local fs, j_states_pos = robot_kine:getPort("JointState"):read()
			local temp_j = {}

			for i=1,robot_arm_joint_count[i] do
				temp_j[i] = pos.positions[i-1]
			end

			armSetPos(OODL,i,temp_j)

			print("Send blocking pose arm")
			print(j_cmd_pos)

		end

		-- BASE --
		if robot_base_count > 0 then		

			local j_cmd_twist = rtt.Variable("geometry_msgs.Twist")
			local twist_base_port_cmd = rttlib.port_clone_conn(oodl_base_serv:getPort("cmd_twist_in"))
			
			twist_base_port_cmd:write(j_cmd_twist)
			print("Send blocking pose base")
			print(j_cmd_twist)
		end

	elseif mode == SIM then

		-- ARM --
		for i=1,robot_arm_count do

			armSetCtrlModes(SIM,i,1)

			local fs, j_states_pos = robot_kine:getPort("JointState"):read()
			local temp_j = {}

			for i=1,robot_arm_joint_count[i] do
				temp_j[i] = pos.positions[i-1]
			end

			armSetPos(SIM,i,temp_j)

			print("Send blocking pose arm")
			print(j_cmd_pos)

		end

		-- BASE --
		if robot_base_count > 0 then

			local j_cmd_twist = rtt.Variable("geometry_msgs.Twist")
			local twist_base_port_cmd = rttlib.port_clone_conn(sim_base_serv:getPort("cmd_twist_in"))
			
			twist_base_port_cmd:write(j_cmd_twist)
			print("Send blocking pose base")
			print(j_cmd_twist)

		end
	end		

end

function switch_to(mode)

	if mode == SIM then
		--set gain cartesian controller
		setK(CARTESIAN_GAIN_SIM)
		--blocking youbot on the last position
		block_robot_position(OODL)
		--visualization mode deactivated
		sim_visual_mode(2)
		--connect queue ports and streams
		queue_output_disconnect()
		queue_input_connect()
		--connect cartesian ports and streams
		kinematic_input_from_sim()
		--activate queue recording
		queue_op_is_loading:send(true)

	elseif mode == HW then
		--set gain cartesian controller
		setK(CARTESIAN_GAIN_OODL)
		--set ARM control mode -> VEL --
		for i=1,robot_arm_count do
			armSetCtrlModes(OODL,i,2)
		end
		--visualization mode activated
		--sim_visual_mode(1)
		--connect queue ports and streams
		queue_input_disconnect()
		--cartesian_goal_disconnect()
		queue_output_connect()
		--connect cartesian ports and streams
		kinematic_input_from_oodl()
		--activate queue downloading
		queue_op_is_loading:send(false)
	end
end

-- DEBUG DEPLOYER FUNCTIONS --

function armSetCtrlModes(stype,arm_index,k)

	if stype == SIM then
		serv = sim_arm_serv[arm_index]
	elseif stype == OODL then
		serv = oodl_arm_serv[arm_index]
	end

	mode_op = serv:getOperation("setControlModesAll")
	mode_op(k)
end

function armSetTor(stype,arm_index,joint_torques)

	armSetCtrlModes(stype,arm_index,3)

	if stype == SIM then
		serv = sim_arm_serv[arm_index]
	elseif stype == OODL then
		serv = oodl_arm_serv[arm_index]
	end

	port = rttlib.port_clone_conn(serv:getPort("joint_effort_command_in"))	
	tor = rtt.Variable("motion_control_msgs.JointEfforts")

	tor.names:resize(robot_arm_joint_count[arm_index])
	tor.efforts:resize(robot_arm_joint_count[arm_index])

	for i=1,robot_arm_joint_count[arm_index] do
		tor.names[i-1] = "arm_joint_"..i
		tor.efforts[i-1] = joint_torques[i]
	end

	port:write(tor)
	print("Send tor")
	print(tor)
end

function armSetVel(stype,arm_index,joint_velocities)

	armSetCtrlModes(stype,arm_index,2)

	if stype == SIM then
		serv = sim_arm_serv[arm_index]
	elseif stype == OODL then
		serv = oodl_arm_serv[arm_index]
	end	

	port = rttlib.port_clone_conn(serv:getPort("joint_velocity_command_in"))	
	vel = rtt.Variable("motion_control_msgs.JointVelocities")

	vel.names:resize(robot_arm_joint_count[arm_index])
	vel.velocities:resize(robot_arm_joint_count[arm_index])

	for i=1,robot_arm_joint_count[arm_index] do
		vel.names[i-1] = "arm_joint_"..i
		vel.velocities[i-1] = joint_velocities[i]
	end

	port:write(vel)
	print("Send vel")
	print(vel)
end

function armSetPos(stype,arm_index,joint_positions)

	armSetCtrlModes(stype,arm_index,1)

	if stype == SIM then
		serv = sim_arm_serv[arm_index]
	elseif stype == OODL then
		serv = oodl_arm_serv[arm_index]
	end

	port = rttlib.port_clone_conn(serv:getPort("joint_position_command_in"))	
	pos = rtt.Variable("motion_control_msgs.JointPositions")

	pos.names:resize(robot_arm_joint_count[arm_index])
	pos.positions:resize(robot_arm_joint_count[arm_index])

	for i=1,robot_arm_joint_count[arm_index] do
		pos.names[i-1] = "arm_joint_"..i
		pos.positions[i-1] = joint_positions[i]
	end

	port:write(pos)
	print("Send pose")
	print(pos)
end

function baseSetCtrlModes(stype,k)

	if stype == SIM then
		serv = sim_base_serv
	elseif stype == OODL then
		serv = oodl_base_serv
	end

	mode_op = serv:getOperation("setControlModesAll")
	mode_op(k)
end

function baseSetTwist(stype,a,b,c)

	baseSetCtrlModes(stype,5)

	if stype == SIM then
		serv = sim_base_serv
	elseif stype == OODL then
		serv = oodl_base_serv
	end

	port = rttlib.port_clone_conn(serv:getPort("cmd_twist_in"))	
	twist = rtt.Variable("geometry_msgs.Twist")
	twist.linear.x = a
	twist.linear.y = b
	twist.linear.z = 0.0
	twist.angular.x = 0.0
	twist.angular.y = 0.0
	twist.angular.z = c
	port:write(twist)
	print("Send twist")
	print(twist)
end

function baseSetTor(stype,a,b,c,d)

	baseSetCtrlModes(stype,3)

	if stype == SIM then
		serv = sim_base_serv
	elseif stype == OODL then
		serv = oodl_base_serv
	end

	port = rttlib.port_clone_conn(serv:getPort("joint_effort_command_in"))	
	tor = rtt.Variable("motion_control_msgs.JointEfforts")
	tor.efforts:fromtab{a,b,c,d}
	tor.names:fromtab{"wheel_joint_fl","wheel_joint_fr","wheel_joint_bl","wheel_joint_br"}
	port:write(tor)
	print("Send tor")
	print(tor)
end

function baseSetVel(stype,a,b,c,d)

	baseSetCtrlModes(stype,2)

	if stype == SIM then
		serv = sim_base_serv
	elseif stype == OODL then
		serv = oodl_base_serv
	end

	port = rttlib.port_clone_conn(serv:getPort("joint_velocity_command_in"))	
	vel = rtt.Variable("motion_control_msgs.JointVelocities")
	vel.velocities:fromtab{a,b,c,d}
	vel.names:fromtab{"wheel_joint_fl","wheel_joint_fr","wheel_joint_bl","wheel_joint_br"}
	port:write(vel)
	print("Send vel")
	print(vel)
end

function baseSetPos(stype,a,b,c,d)

	baseSetCtrlModes(stype,1)

	if stype == SIM then
		serv = sim_base_serv
	elseif stype == OODL then
		serv = oodl_base_serv
	end

	port = rttlib.port_clone_conn(serv:getPort("joint_position_command_in"))	
	pos = rtt.Variable("motion_control_msgs.JointPositions")
	pos.positions:fromtab{a,b,c,d}
	pos.names:fromtab{"wheel_joint_fl","wheel_joint_fr","wheel_joint_bl","wheel_joint_br"}
	port:write(pos)
	print("Send pose")
	print(pos)
end

function gripSetStat(stype,gripper_index,a)

	if stype == SIM then
		serv = sim_grip_serv[gripper_index]
	elseif stype == OODL then
		serv = oodl_grip_serv[gripper_index]
	end

	port = rttlib.port_clone_conn(serv:getPort("gripper_cmd_position_in"))	
	pos = rtt.Variable("motion_control_msgs.JointPositions")
	pos.positions:fromtab{a}
	pos.names:fromtab{"gripper_pos"}
	port:write(pos)
	print("Send pose")
	print(pos)
end

-- CARTESIAN CONTROLLER FUNCTIONS -- 

function setK(gain)
	robot_ctrl_cartesian:stop()
	K:fromtab{gain,gain,gain,gain,gain,gain}
	robot_ctrl_cartesian:configure()
	robot_ctrl_cartesian:start()
	print(K)
end

function set_cartesian_goal(dx, dy, dz)
	
	desiredPosPort = rttlib.port_clone_conn(robot_ctrl_cartesian:getPort("CartesianDesiredPosition"))
	queueCartPort = rttlib.port_clone_conn(cmd_queue:getPort("ros_cartesian_command_in"))	
	fs, startPos = robot_ctrl_cartesian:getPort("CartesianSensorPosition"):read()

	startPos.position.x = startPos.position.x + dx
	startPos.position.y = startPos.position.y + dy
	startPos.position.z = startPos.position.z + dz

	print("New setpoint EE")
	print(startPos)
	desiredPosPort:write(startPos)
	queueCartPort:write(startPos)
end

function readSensPos()
	fs, pos = robot_ctrl_cartesian:getPort("CartesianSensorPosition"):read()	
	print(pos)
end
