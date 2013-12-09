require "rttlib"
require "rfsm_rtt"
require "rttros"
require "kdlpp"
require "kdlutils"

require "complete"
require "readline"

require "definitions"
require "connection"

--coroutine.create(udp_server)

SIM, HW, BOTH, LUA_DEPLOYER, OPS_DEPLOYER, VREP, OODL = 0, 1, 2, 3, 4, 5 ,6

rttlib.color = true
run_status = HW
deployer_type = LUA_DEPLOYER

-- Lua deployer
if deployer_type == LUA_DEPLOYER then

	tc = rtt.getTC()
	depl = tc:getPeer("Deployer")

-- Ops deployer
elseif deployer_type == OPS_DEPLOYER then

	tc=rtt.getTC()
	tcName=tc:getName()

	if tcName=="lua" then
	depl=tc:getPeer("deployer")
	elseif tcName=="Deployer" then
	depl = tc
	end

	depl:loadComponent("Supervisor", "OCL::LuaComponent")

	if tcName == "lua" then
	depl:aliasPeer("Supervisor", "deployer", "Deployer")
	elseif tcName == "Deployer" then
	depl:addPeer("Supervisor", "Deployer")
	end

end

-- Import component
depl:import("fbsched")
depl:import("YouBot_VREP")
depl:import("YouBot_OODL")
depl:import("kdl_typekit")
depl:import("rtt_sensor_msgs")
depl:import("rtt_std_msgs")
depl:import("rtt_geometry_msgs")
depl:import("rtt_nav_msgs")
depl:import("rtt_motion_control_msgs")
depl:import("rtt_rosnode")

-- Loading component
depl:loadComponent("controlloop_scheduler", "FBSched")
depl:loadComponent("YouBot_OODL", "YouBot::YouBotOODL")
depl:loadComponent("YouBot_VREP", "YouBot::YouBotVREP") 

-- Getting peers of components
controlloop_scheduler = depl:getPeer("controlloop_scheduler")
youbot_vrep = depl:getPeer("YouBot_VREP")
youbot_oodl = depl:getPeer("YouBot_OODL")

-- Using fbsched for activity
depl:setActivity("controlloop_scheduler",0.002,99,rtt.globals.ORO_SCHED_RT)
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_VREP")
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_OODL")

-- Creating connections policy
cp = rtt.Variable('ConnPolicy')
cp.type = rtt.globals.DATA   -- type data

-- Launching scheduler
print("Starting control loop")
controlloop_scheduler:configure()
controlloop_scheduler:start()

-- SIM MODE
if run_status == SIM then

	rtt.logl('Info', "Youbot VREP configure.")
	youbot_vrep:configure()

	vrep_arm_serv = youbot_vrep:provides("Arm1")
	vrep_base_serv = youbot_vrep:provides("Base")
	vrep_grip_serv = youbot_vrep:provides("Gripper1")

	vrep_arm_op_clear = vrep_arm_serv:getOperation("clearControllerTimeouts")
	vrep_arm_op_stat = vrep_arm_serv:getOperation("displayMotorStatuses")

	vrep_base_op_clear = vrep_base_serv:getOperation("clearControllerTimeouts")
	vrep_base_op_stat = vrep_base_serv:getOperation("displayMotorStatuses")

	-- ROS simulated robot streaming
	depl:stream("YouBot_VREP.Arm1.in_joint_state",rtt.provides("ros"):topic("/vrep/arm1/joint_states"))
	depl:stream("YouBot_VREP.Base.in_joint_state",rtt.provides("ros"):topic("/vrep/base/joint_states"))
	depl:stream("YouBot_VREP.Base.in_odometry_state",rtt.provides("ros"):topic("/odom"))

	depl:stream("YouBot_VREP.Arm1.out_joint_position_command",rtt.provides("ros"):topic("/arm_1/arm_controller/position_command"))
	depl:stream("YouBot_VREP.Arm1.out_joint_velocity_command",rtt.provides("ros"):topic("/arm_1/arm_controller/velocity_command"))
	depl:stream("YouBot_VREP.Arm1.out_joint_effort_command",rtt.provides("ros"):topic("/arm_1/arm_controller/force_command"))
	depl:stream("YouBot_VREP.Base.out_joint_position_command",rtt.provides("ros"):topic("/base/base_controller/position_command"))
	depl:stream("YouBot_VREP.Base.out_joint_velocity_command",rtt.provides("ros"):topic("/base/base_controller/velocity_command"))
	depl:stream("YouBot_VREP.Base.out_joint_effort_command",rtt.provides("ros"):topic("/base/base_controller/force_command"))
	depl:stream("YouBot_VREP.Base.out_cmd_twist",rtt.provides("ros"):topic("/cmd_vel"))
	depl:stream("YouBot_VREP.Gripper1.out_gripper_cmd_position",rtt.provides("ros"):topic("/arm_1/gripper_controller/position_command"))

	rtt.logl('Info', "Youbot VREP start.")
	youbot_vrep:start()

-- HW MODE
elseif run_status == HW then
	
	rtt.logl('Info', "Youbot OODL configure.")
	youbot_oodl:configure()

	oodl_arm_serv = youbot_oodl:provides("Arm1")
	oodl_base_serv = youbot_oodl:provides("Base")
	oodl_grip_serv = youbot_oodl:provides("Gripper1")

	oodl_arm_op_clear = oodl_arm_serv:getOperation("clearControllerTimeouts")
	oodl_arm_op_stat = oodl_arm_serv:getOperation("displayMotorStatuses")

	oodl_base_op_clear = oodl_base_serv:getOperation("clearControllerTimeouts")
	oodl_base_op_stat = oodl_base_serv:getOperation("displayMotorStatuses")

	rtt.logl('Info', "Youbot OODL start.")
	youbot_oodl:start()

-- SIM+HW MODE
elseif run_status == BOTH then

end

-- Functions defintion
function armSetCtrlModes(stype,k)

	if stype == VREP then
		serv = vrep_arm_serv
	elseif stype == OODL then
		serv = oodl_arm_serv
	end

	mode_op = serv:getOperation("setControlModesAll")
	mode_op(k)
end

function armSetTor(stype,a,b,c,d,e)

	if stype == VREP then
		serv = vrep_arm_serv
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

	if stype == VREP then
		serv = vrep_arm_serv
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

	if stype == VREP then
		serv = vrep_arm_serv
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

	if stype == VREP then
		serv = vrep_base_serv
	elseif stype == OODL then
		serv = oodl_base_serv
	end

	mode_op = serv:getOperation("setControlModesAll")
	mode_op(k)
end

function baseSetTwist(stype,a,b,c)

	if stype == VREP then
		serv = vrep_base_serv
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

	if stype == VREP then
		serv = vrep_base_serv
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

	if stype == VREP then
		serv = vrep_base_serv
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

	if stype == VREP then
		serv = vrep_base_serv
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
