-- Enum definition
SIM, HW, BOTH, LUA_DEPLOYER, OPS_DEPLOYER, VREP, OODL, REMOTE, LOCAL = 0, 1, 2, 3, 4, 5 ,6, 7, 8

hash_config = {}
hash_config["SIM"] = SIM
hash_config["HW"] = HW
hash_config["BOTH"] = BOTH
hash_config["LUA_DEPLOYER"] = LUA_DEPLOYER
hash_config["OPS_DEPLOYER"] = OPS_DEPLOYER
hash_config["VREP"] = VREP
hash_config["OODL"] = OODL
hash_config["REMOTE"] = REMOTE
hash_config["LOCAL"] = LOCAL

rttlib.color = true
running = true

-- HILAS SETUP --
configTable = inifile.parse('config/hilas.ini')

robot_name = configTable['robot']['name']
robot_arm_count = configTable['robot']['armCount']
robot_base_count = configTable['robot']['baseCount']

run_status = hash_config[configTable['hilas']['runMode']] --SIM
deployer_type = hash_config[configTable['hilas']['deployerMode']] --LUA_DEPLOYER
communication_type = hash_config[configTable['hilas']['usageMode']] --DEBUG
is_ros_enabled = configTable['hilas']['useROS']

CARTESIAN_GAIN_SIM = configTable['cartesian_controller']['gain_hw']
CARTESIAN_GAIN_OODL = configTable['cartesian_controller']['gain_sim']

TOPIC_ARM_POSITION_COMMAND = configTable['ROS']['topicArmPosCommand']
TOPIC_ARM_VELOCITY_COMMAND = configTable['ROS']['topicArmVelCommand']
TOPIC_ARM_EFFORT_COMMAND = configTable['ROS']['topicArmTorCommand']
TOPIC_BASE_TWIST_COMMAND = configTable['ROS']['topicBaseTwistCommand']
TOPIC_BASE_ODOM_STATE = configTable['ROS']['topicBaseOdomState']
TOPIC_GRIPPER_POSITION_COMMAND = configTable['ROS']['topicGripperPosCommand']
SIM_TOPIC_ARM_JOINT_STATES_RX = configTable['ROS']['topicSimArmJointState']
SIM_TOPIC_BASE_JOINT_STATES_RX = configTable['ROS']['topicSimBaseJointState']
SIM_TOPIC_ODOM_STATE_RX = configTable['ROS']['topicSimOdomState']

-- USAGE DEPLOYER FUNCTIONS --

function sim_visual_mode(a)

	--visModePort = sim_vismode:getPort("visMode")
	--x = rtt.Variable("/std_msgs/Int32")
	--x.data = a
	--visModePort:write(x)
	sim_set_mode(a)

end

function vel_startup()

	armSetCtrlModes(OODL,2)
	baseSetCtrlModes(OODL,5)
	cartesian_controller_start()

end

-- Definition setup functions
function simulation_setup()

	rtt.logl('Info', "Youbot SIM configure.")
	youbot_sim:configure()

	sim_set_mode = youbot_sim:getOperation("setSimMode")

	sim_arm_serv = youbot_sim:provides("Arm1")
	sim_base_serv = youbot_sim:provides("Base")
	sim_grip_serv = youbot_sim:provides("Gripper1")

	sim_arm_op_clear = sim_arm_serv:getOperation("clearControllerTimeouts")
	sim_arm_op_stat = sim_arm_serv:getOperation("displayMotorStatuses")

	sim_base_op_clear = sim_base_serv:getOperation("clearControllerTimeouts")
	sim_base_op_stat = sim_base_serv:getOperation("displayMotorStatuses")

	--require "definitions"
end

function queue_setup()

	youbot_queue:configure()
	youbot_queue:start()
	queue_op_is_loading = youbot_queue:getOperation("setIsInLoading")
	-- TEST --
	depl:stream("YouBot_QUEUE.out_ros_cartesian_command", rtt.provides("ros"):topic("/cart_debug"))
end

function queue_output_disconnect()

	youbot_queue:getPort("out_arm_joint_position_command"):disconnect()
	youbot_queue:getPort("out_arm_joint_velocity_command"):disconnect()
	youbot_queue:getPort("out_arm_joint_effort_command"):disconnect()
	youbot_queue:getPort("out_base_cmd_twist"):disconnect()
	youbot_queue:getPort("out_gripper_joint_position_command"):disconnect()
	youbot_queue:getPort("out_ros_planner_command"):disconnect()
	youbot_queue:getPort("out_ros_cartesian_command"):disconnect()

end

function queue_input_connect()

	--depl:stream("YouBot_QUEUE.ros_arm_joint_position_command", rtt.provides("ros"):topic(TOPIC_ARM_POSITION_COMMAND))
	--depl:stream("YouBot_QUEUE.ros_arm_joint_velocity_command", rtt.provides("ros"):topic(TOPIC_ARM_VELOCITY_COMMAND))
	--depl:stream("YouBot_QUEUE.ros_arm_joint_effort_command", rtt.provides("ros"):topic(TOPIC_ARM_EFFORT_COMMAND))
	--depl:stream("YouBot_QUEUE.ros_base_cmd_twist", rtt.provides("ros"):topic(TOPIC_BASE_TWIST_COMMAND))
	--depl:stream("YouBot_QUEUE.ros_gripper_joint_position_command", rtt.provides("ros"):topic(TOPIC_GRIPPER_POSITION_COMMAND))
	depl:stream("YouBot_QUEUE.ros_planner_command", rtt.provides("ros"):topic("/move_base_simple/goal"))
	depl:connect("YouBot_QUEUE.ros_cartesian_command","MOVE_OUT.moveOut", cp)
	--depl:stream("YouBot_QUEUE.ros_cartesian_command", rtt.provides("ros"):topic("/youbot/desired_ee"))

end

function queue_input_disconnect()

	youbot_queue:getPort("ros_arm_joint_position_command"):disconnect()
	youbot_queue:getPort("ros_arm_joint_velocity_command"):disconnect()
	youbot_queue:getPort("ros_arm_joint_effort_command"):disconnect()
	youbot_queue:getPort("ros_base_cmd_twist"):disconnect()
	youbot_queue:getPort("ros_gripper_joint_position_command"):disconnect()
	youbot_queue:getPort("ros_planner_command"):disconnect()
	youbot_queue:getPort("ros_cartesian_command"):disconnect()

end

function queue_output_connect()

	depl:connect("YouBot_QUEUE.out_arm_joint_position_command","YouBot_OODL.Arm1.joint_position_command", cp)
	depl:connect("YouBot_QUEUE.out_arm_joint_velocity_command","YouBot_OODL.Arm1.joint_velocity_command", cp)
	depl:connect("YouBot_QUEUE.out_arm_joint_effort_command", "YouBot_OODL.Arm1.joint_effort_command", cp)
	depl:connect("YouBot_QUEUE.out_base_cmd_twist", "YouBot_OODL.Base.cmd_twist", cp)
	depl:connect("YouBot_QUEUE.out_gripper_joint_position_command", "YouBot_OODL.Gripper1.gripper_cmd_position", cp)
	depl:stream("YouBot_QUEUE.out_ros_planner_command", rtt.provides("ros"):topic("/move_base_simple/goal")) -- PLANNER DA INSERIRE
	depl:connect("YouBot_QUEUE.out_ros_cartesian_command", "YouBot_CTRL_CARTESIAN.CartesianDesiredPosition", cp)
	depl:connect("YouBot_QUEUE.from_cartesian_status", "YouBot_KINE.EEPose", cp)
	
	--depl:stream("YouBot_QUEUE.out_ros_cartesian_command", rtt.provides("ros"):topic("/cart_debug"))

end

function cartesian_controller_setup()

	depl:connect("YouBot_KINE.EEPose","YouBot_CTRL_CARTESIAN.CartesianSensorPosition",cp)
	depl:connect("YouBot_KINE.EETwistRTT","YouBot_CTRL_CARTESIAN.CartesianOutputVelocity",cp)

	--depl:loadService("YouBot_KINE","rosparam")

	--youbot_kine:provides("rosparam"):refreshProperty("robot_description",false,false)
	K = youbot_ctrl_cartesian:getProperty("K")
	local gain = 0.02
	K:fromtab{gain,gain,gain,gain,gain,gain}

	depl:stream("YouBot_KINE.EEPose",rtt.provides("ros"):topic("/youbot/EEPose"))
	cartesian_goal_connect()
end

function cartesian_controller_start()

	youbot_kine:configure()
	youbot_ctrl_cartesian:configure()
	youbot_kine:start()
	youbot_ctrl_cartesian:start()

	depl:connect("YouBot_CTRL_CARTESIAN.CartesianDesiredPosition", "MOVE_OUT.moveOut", cp)

	desiredPosPort = rttlib.port_clone_conn(youbot_ctrl_cartesian:getPort("CartesianDesiredPosition"))
	--print("Configure JointSpaceWeights")
	js_weight_port = rttlib.port_clone_conn(youbot_kine:getPort("JointSpaceWeights"))
	js_weight = rtt.Variable("float64[]")
	js_weight:resize(8)
	js_weight:fromtab{1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0}
	js_weight_port:write(js_weight)
	fs, startPos = youbot_ctrl_cartesian:getPort("CartesianSensorPosition"):read()
	print("Start Position of Controller")
	print(startPos)
	desiredPosPort:write(startPos)

end

function cartesian_goal_connect()

	--depl:stream("YouBot_CTRL_CARTESIAN.CartesianDesiredPosition",rtt.provides("ros"):topic("/youbot/desired_ee"))
	depl:stream("YouBot_CTRL_CARTESIAN.CartesianDesiredPosition",rtt.provides("ros"):topic("/interactiveEEPose"))

end

function cartesian_goal_disconnect()

	youbot_ctrl_cartesian:getPort("CartesianDesiredPosition"):disconnect()

end


function cartesian_input_from_vrep()

	depl:connect("YouBot_KINE.JointState","youbot_sim.Arm1.joint_state",cp)
	depl:connect("YouBot_KINE.JointVelocities","youbot_sim.Arm1.joint_velocity_command",cp)
	depl:connect("YouBot_KINE.BaseTwist","youbot_sim.Base.cmd_twist",cp)
	depl:connect("YouBot_KINE.BaseOdom","youbot_sim.Base.odometry_state",cp)

	--depl:stream("YouBot_KINE.JointState",rtt.provides("ros"):topic("/vrep/arm_1/joint_states"))
	--depl:stream("YouBot_KINE.JointVelocities",rtt.provides("ros"):topic(TOPIC_ARM_VELOCITY_COMMAND))
	--depl:stream("YouBot_KINE.BaseTwist",rtt.provides("ros"):topic(TOPIC_BASE_TWIST_COMMAND))
	--depl:stream("YouBot_KINE.BaseOdom",rtt.provides("ros"):topic(TOPIC_BASE_ODOM_STATE))

end

function cartesian_input_from_oodl()

	depl:connect("YouBot_KINE.JointState","YouBot_OODL.Arm1.joint_state",cp)
	depl:connect("YouBot_KINE.JointVelocities","YouBot_OODL.Arm1.joint_velocity_command",cp)
	depl:connect("YouBot_KINE.BaseTwist","YouBot_OODL.Base.cmd_twist",cp)
	depl:connect("YouBot_KINE.BaseOdom","YouBot_OODL.Base.odometry_state",cp)

end

function youbot_republisher_oodl()

	depl:connect("YouBotStateRepublisher.arm_state","YouBot_OODL.Arm1.joint_state",cp)
	depl:connect("YouBotStateRepublisher.base_state","YouBot_OODL.Base.joint_state",cp)
	depl:stream("YouBotStateRepublisher.youbot_state",rtt.provides("ros"):topic("/joint_states"))
	depl:stream("YouBotStateRepublisher.odometry_state",rtt.provides("ros"):topic("/odom"))
end

--function connect_sim_ros_streams()
	-- ROS simulated robot streaming
	--depl:stream("youbot_sim.Arm1.in_joint_state",rtt.provides("ros"):topic("/vrep/arm_1/joint_states"))
	--depl:stream("youbot_sim.Base.in_joint_state",rtt.provides("ros"):topic("/vrep/base/joint_states"))
	--depl:stream("youbot_sim.Base.in_odometry_state",rtt.provides("ros"):topic("/odom"))

	--depl:stream("youbot_sim.Arm1.out_joint_position_command",rtt.provides("ros"):topic(TOPIC_ARM_POSITION_COMMAND))
	--depl:stream("youbot_sim.Arm1.out_joint_velocity_command",rtt.provides("ros"):topic(TOPIC_ARM_VELOCITY_COMMAND))
	--depl:stream("youbot_sim.Arm1.out_joint_effort_command",rtt.provides("ros"):topic(TOPIC_ARM_EFFORT_COMMAND))
	--depl:stream("youbot_sim.Base.out_joint_position_command",rtt.provides("ros"):topic("/base/base_controller/position_command"))
	--depl:stream("youbot_sim.Base.out_joint_velocity_command",rtt.provides("ros"):topic("/base/base_controller/velocity_command"))
	--depl:stream("youbot_sim.Base.out_joint_effort_command",rtt.provides("ros"):topic("/base/base_controller/force_command"))
	--depl:stream("youbot_sim.Base.out_cmd_twist",rtt.provides("ros"):topic(TOPIC_BASE_TWIST_COMMAND))
	--depl:stream("youbot_sim.Gripper1.out_gripper_cmd_position",rtt.provides("ros"):topic(TOPIC_GRIPPER_POSITION_COMMAND))
--end

function disconnect_sim_ros_streams() 
	--ARM 
	sim_arm_serv:getPort("in_joint_state"):disconnect()
	sim_arm_serv:getPort("out_joint_position_command"):disconnect()
	sim_arm_serv:getPort("out_joint_velocity_command"):disconnect()
	sim_arm_serv:getPort("out_joint_effort_command"):disconnect()
	--BASE
	sim_base_serv:getPort("out_joint_position_command"):disconnect()
	sim_base_serv:getPort("out_joint_velocity_command"):disconnect()
	sim_base_serv:getPort("out_joint_effort_command"):disconnect()
	sim_base_serv:getPort("out_cmd_twist"):disconnect()
	sim_base_serv:getPort("in_joint_state"):disconnect()
	sim_base_serv:getPort("in_odometry_state"):disconnect()
	--GRIPPER
	sim_grip_serv:getPort("out_gripper_cmd_position"):disconnect()
end

function oodl_setup()

	rtt.logl('Info', "Youbot OODL configure.")
	youbot_oodl:configure()

	oodl_arm_serv = youbot_oodl:provides("Arm1")
	oodl_base_serv = youbot_oodl:provides("Base")
	oodl_grip_serv = youbot_oodl:provides("Gripper1")

	oodl_arm_op_clear = oodl_arm_serv:getOperation("clearControllerTimeouts")
	oodl_arm_op_stat = oodl_arm_serv:getOperation("displayMotorStatuses")

	oodl_base_op_clear = oodl_base_serv:getOperation("clearControllerTimeouts")
	oodl_base_op_stat = oodl_base_serv:getOperation("displayMotorStatuses")

	--require "definitions"

end

function connect_oodl_ros_streams()
	--##### ROS streams ######
	--ARM command 
	depl:stream("YouBot_OODL.Arm1.joint_position_command",rtt.provides("ros"):topic(TOPIC_ARM_POSITION_COMMAND))
	depl:stream("YouBot_OODL.Arm1.joint_velocity_command",rtt.provides("ros"):topic(TOPIC_ARM_VELOCITY_COMMAND))
	depl:stream("YouBot_OODL.Arm1.joint_effort_command",rtt.provides("ros"):topic(TOPIC_ARM_EFFORT_COMMAND))
	
	--Base command
	depl:stream("YouBot_OODL.Base.cmd_twist",rtt.provides("ros"):topic(TOPIC_BASE_TWIST_COMMAND))
	--Odometry state
	depl:stream("YouBot_OODL.Base.odometry_state",rtt.provides("ros"):topic(TOPIC_BASE_ODOM_STATE))

	--Gripper command
	depl:stream("YouBot_OODL.Gripper1.gripper_cmd_position",rtt.provides("ros"):topic(TOPIC_GRIPPER_POSITION_COMMAND))

	--SIM streams (VISUALIZATION_MODE)
    depl:stream("YouBot_OODL.Base.odometry_state",rtt.provides("ros"):topic(SIM_TOPIC_ODOM_STATE_RX))
    depl:stream("YouBot_OODL.Base.joint_state",rtt.provides("ros"):topic(SIM_TOPIC_BASE_JOINT_STATES_RX))
    depl:stream("YouBot_OODL.Arm1.joint_state",rtt.provides("ros"):topic(SIM_TOPIC_ARM_JOINT_STATES_RX))

	--#### END ROS streams ######
end

function disconnect_oodl_ros_streams() 
	--ARM
	oodl_arm_serv:getPort("joint_position_command"):disconnect()
	oodl_arm_serv:getPort("joint_velocity_command"):disconnect()
	oodl_arm_serv:getPort("joint_effort_command"):disconnect()
	oodl_arm_serv:getPort("joint_state"):disconnect()
	--BASE
	oodl_base_serv:getPort("cmd_twist"):disconnect()
	oodl_base_serv:getPort("odometry_state"):disconnect()
	oodl_base_serv:getPort("joint_state"):disconnect()
	--GRIPPER
	oodl_grip_serv:getPort("gripper_cmd_position"):disconnect()
end

function block_youbot_position(mode)

	if mode == OODL then

		-- ARM --
		armSetCtrlModes(OODL,1)

		local fs, j_states_pos = youbot_kine:getPort("JointState"):read()

		local pos_port_cmd = rttlib.port_clone_conn(oodl_arm_serv:getPort("joint_position_command"))
		local j_cmd_pos = rtt.Variable("motion_control_msgs.JointPositions")

		j_cmd_pos.names:fromtab{"arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"}
		j_cmd_pos.positions:fromtab{j_states_pos.position[0],j_states_pos.position[1],j_states_pos.position[2],j_states_pos.position[3],j_states_pos.position[4]}

		pos_port_cmd:write(j_cmd_pos)
		print("Send blocking pose arm")
		print(j_cmd_pos)

		-- BASE --
		local j_cmd_twist = rtt.Variable("geometry_msgs.Twist")
		local twist_base_port_cmd = rttlib.port_clone_conn(oodl_base_serv:getPort("cmd_twist"))
		
		twist_base_port_cmd:write(j_cmd_twist)
		print("Send blocking pose base")
		print(j_cmd_twist)

	elseif mode == SIM then

		-- ARM --
		armSetCtrlModes(SIM,1)

		local fs, j_states_pos = youbot_kine:getPort("JointState"):read()

		local pos_port_cmd = rttlib.port_clone_conn(sim_arm_serv:getPort("joint_position_command"))
		local j_cmd_pos = rtt.Variable("motion_control_msgs.JointPositions")

		j_cmd_pos.names:fromtab{"arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"}
		j_cmd_pos.positions:fromtab{j_states_pos.position[0],j_states_pos.position[1],j_states_pos.position[2],j_states_pos.position[3],j_states_pos.position[4]}

		pos_port_cmd:write(j_cmd_pos)
		print("Send blocking pose arm")
		print(j_cmd_pos)

		-- BASE --
		local j_cmd_twist = rtt.Variable("geometry_msgs.Twist")
		local twist_base_port_cmd = rttlib.port_clone_conn(sim_base_serv:getPort("cmd_twist"))
		
		twist_base_port_cmd:write(j_cmd_twist)
		print("Send blocking pose base")
		print(j_cmd_twist)

	end		

end

function switch_to(mode)

	if mode == SIM then
		--set gain cartesian controller
		setK(CARTESIAN_GAIN_SIM)
		--blocking youbot on the last position
		block_youbot_position(OODL)
		--visualization mode deactivated
		sim_visual_mode(0)
		--disconnect HW ports and streams
		disconnect_oodl_ros_streams()
		--connect SIM ports and streams
		connect_sim_ros_streams()
		--connect queue ports and streams
		queue_output_disconnect()
		queue_input_connect()
		--connect cartesian ports and streams
		cartesian_input_from_vrep()
		--activate queue recording
		queue_op_is_loading:send(true)

	elseif mode == HW then
		--set gain cartesian controller
		setK(CARTESIAN_GAIN_OODL)
		--set ARM control mode -> VEL --
		armSetCtrlModes(OODL,2)
		--visualization mode activated
		sim_visual_mode(1)
		--disconnect SIM ports and streams
		disconnect_sim_ros_streams()
		--connect HW ports and streams
		connect_oodl_ros_streams()
		--connect queue ports and streams
		queue_input_disconnect()
		--cartesian_goal_disconnect()
		queue_output_connect()
		--connect cartesian ports and streams
		cartesian_input_from_oodl()
		--activate queue downloading
		queue_op_is_loading:send(false)
	end
end

-- DEBUG DEPLOYER FUNCTIONS --

function armSetCtrlModes(stype,k)

	if stype == SIM then
		serv = sim_arm_serv
	elseif stype == OODL then
		serv = oodl_arm_serv
	end

	mode_op = serv:getOperation("setControlModesAll")
	mode_op(k)
end

function armSetTor(stype,a,b,c,d,e)

	armSetCtrlModes(stype,3)

	if stype == SIM then
		serv = sim_arm_serv
	elseif stype == OODL then
		serv = oodl_arm_serv
	end

	port = rttlib.port_clone_conn(serv:getPort("joint_effort_command"))	
	tor = rtt.Variable("motion_control_msgs.JointEfforts")
	tor.efforts:fromtab{a,b,c,d,e}
	tor.names:fromtab{"arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"}
	port:write(tor)
	print("Send tor")
	print(tor)
end

function armSetVel(stype,a,b,c,d,e)

	armSetCtrlModes(stype,2)

	if stype == SIM then
		serv = sim_arm_serv
	elseif stype == OODL then
		serv = oodl_arm_serv
	end	

	port = rttlib.port_clone_conn(serv:getPort("joint_velocity_command"))	
	vel = rtt.Variable("motion_control_msgs.JointVelocities")
	vel.velocities:fromtab{a,b,c,d,e}
	vel.names:fromtab{"arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"}
	port:write(vel)
	print("Send vel")
	print(vel)
end

function armSetPos(stype,a,b,c,d,e)

	armSetCtrlModes(stype,1)

	if stype == SIM then
		serv = sim_arm_serv
	elseif stype == OODL then
		serv = oodl_arm_serv
	end

	port = rttlib.port_clone_conn(serv:getPort("joint_position_command"))	
	pos = rtt.Variable("motion_control_msgs.JointPositions")
	pos.positions:fromtab{a,b,c,d,e}
	pos.names:fromtab{"arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"}
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

	port = rttlib.port_clone_conn(serv:getPort("cmd_twist"))	
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

	port = rttlib.port_clone_conn(serv:getPort("joint_effort_command"))	
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

	port = rttlib.port_clone_conn(serv:getPort("joint_velocity_command"))	
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

	port = rttlib.port_clone_conn(serv:getPort("joint_position_command"))	
	pos = rtt.Variable("motion_control_msgs.JointPositions")
	pos.positions:fromtab{a,b,c,d}
	pos.names:fromtab{"wheel_joint_fl","wheel_joint_fr","wheel_joint_bl","wheel_joint_br"}
	port:write(pos)
	print("Send pose")
	print(pos)
end

function gripSetStat(stype,a)

	if stype == SIM then
		serv = sim_grip_serv
	elseif stype == OODL then
		serv = oodl_grip_serv
	end

	port = rttlib.port_clone_conn(serv:getPort("gripper_cmd_position"))	
	pos = rtt.Variable("motion_control_msgs.JointPositions")
	pos.positions:fromtab{a}
	pos.names:fromtab{"gripper_pos"}
	port:write(pos)
	print("Send pose")
	print(pos)
end

-- CARTESIAN CONTROLLER FUNCTIONS -- 

function setK(gain)
  youbot_ctrl_cartesian:stop()
  K:fromtab{gain,gain,gain,gain,gain,gain}
  youbot_ctrl_cartesian:configure()
  youbot_ctrl_cartesian:start()
  print(K)
end

function move(dx, dy, dz)
  fs, startPos = youbot_ctrl_cartesian:getPort("CartesianSensorPosition"):read()
  startPos.position.x = startPos.position.x + dx
  startPos.position.y = startPos.position.y + dy
  startPos.position.z = startPos.position.z + dz
  -- startPos.orientation.x = 0.7071067811865476;
  -- startPos.orientation.y = 0;
  -- startPos.orientation.z = 0;
  -- startPos.orientation.w = -0.7071067811865476;
  --desiredPosPort:write(startPos)
  port = move_out:getPort("moveOut")
  port:write(startPos)
  print("New setpoint EE")
end

function readSensPos()
  fs, pos = youbot_ctrl_cartesian:getPort("CartesianSensorPosition"):read()	
  print(pos)
end