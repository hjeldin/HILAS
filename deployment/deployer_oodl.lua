require "rttlib"
require "rfsm_rtt"
require "rttros"
require "kdlpp"
require "kdlutils"

require "complete"
require "readline"

require "definitions"
require "connection"

coroutine.create(udp_server)

rttlib.color = true
run_status = SIM
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
	depl:stream("Driver_VREP.in_joint_states",rtt.provides("ros"):topic("/joint_states"))
	depl:stream("Driver_VREP.in_odometry_state",rtt.provides("ros"):topic("/odom"))

	depl:stream("YouBot_VREP.Arm1.out_joint_position_command",rtt.provides("ros"):topic("/arm_1/arm_controller/position_command"))
	depl:stream("YouBot_VREP.Arm1.out_joint_velocity_command",rtt.provides("ros"):topic("/arm_1/arm_controller/velocity_command"))
	depl:stream("YouBot_VREP.Arm1.out_joint_effort_command",rtt.provides("ros"):topic("/arm_1/arm_controller/force_command"))
	depl:stream("YouBot_VREP.Base.out_joint_position_command",rtt.provides("ros"):topic("/base/base_controller/position_command"))
	depl:stream("YouBot_VREP.Base.out_joint_velocity_command",rtt.provides("ros"):topic("/base/base_controller/velocity_command"))
	depl:stream("YouBot_VREP.Base.out_joint_effort_command",rtt.provides("ros"):topic("/base/base_controller/force_command"))
	depl:stream("YouBot_VREP.Base.out_cmd_twist",rtt.provides("ros"):topic("/cmd_vel"))
	depl:stream("YouBot_VREP.Gripper1.out_position_command",rtt.provides("ros"):topic("/arm_1/gripper_controller/position_command"))

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