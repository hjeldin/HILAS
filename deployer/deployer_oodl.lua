package.path = package.path .. ';./include/?.lua'
require "rttlib"
--require "rfsm_rtt"
require "rttros"
--require "kdlpp"
--require "kdlutils"
--require "complete"
--require "readline"
require 'inifile'
require "definitions"

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
--depl:import("kdl_typekit")
depl:import("rtt_roscomm")
depl:import("rtt_ros")
depl:import("rtt_sensor_msgs")
depl:import("rtt_std_msgs")
depl:import("rtt_geometry_msgs")
depl:import("rtt_nav_msgs")
depl:import("rtt_motion_control_msgs")

if is_ros_enabled == true then
	depl:import("rtt_rosnode")
end

depl:import(string.lower(robot_name).."_sim")
depl:import(string.lower(robot_name).."_oodl")
depl:import(string.lower(robot_name).."_kinematics")
depl:import(string.lower(robot_name).."_republisher")
depl:import(string.lower(robot_name).."_cmddemux")

depl:import("cmd_queue")
depl:import("cartesian_motion_control")

-- OLD DEPLOYER
-- -- Loading component
depl:loadComponent("controlloop_scheduler", "FBSched")
depl:loadComponent("Robot_OODL", firstToUpper(robot_name).."::"..firstToUpper(robot_name).."OODL")
depl:loadComponent("Robot_SIM", firstToUpper(robot_name).."::"..firstToUpper(robot_name).."SIM") 
depl:loadComponent("Cmd_QUEUE", "Cmd_queue")

depl:loadComponent("Robot_KINE", firstToUpper(robot_name).."::"..firstToUpper(robot_name).."Kinematics")	
depl:loadComponent("Robot_CTRL_CARTESIAN", "MotionControl::CartesianControllerPos")
depl:loadComponent("Robot_STATE_PUBLISHER", firstToUpper(robot_name).."::"..firstToUpper(robot_name).."StateRepublisher")
depl:loadComponent("Robot_CMDDEMUX", firstToUpper(robot_name).."::"..firstToUpper(robot_name).."CmdDemux")

-- Getting peers of components
controlloop_scheduler = depl:getPeer("controlloop_scheduler")
robot_sim = depl:getPeer("Robot_SIM")
robot_oodl = depl:getPeer("Robot_OODL")
cmd_queue = depl:getPeer("Cmd_QUEUE")

robot_kine = depl:getPeer("Robot_KINE")
robot_ctrl_cartesian = depl:getPeer("Robot_CTRL_CARTESIAN")
robot_repub = depl:getPeer("Robot_STATE_PUBLISHER")
robot_cmddemux = depl:getPeer("Robot_CMDDEMUX")

-- Using fbsched for activity
depl:setActivity("controlloop_scheduler",0.002,99,rtt.globals.ORO_SCHED_OTHER)
depl:setMasterSlaveActivity("controlloop_scheduler","Robot_SIM")
depl:setMasterSlaveActivity("controlloop_scheduler","Robot_OODL")
depl:setMasterSlaveActivity("controlloop_scheduler","Cmd_QUEUE")

depl:setMasterSlaveActivity("controlloop_scheduler","Robot_KINE")
depl:setMasterSlaveActivity("controlloop_scheduler","Robot_CTRL_CARTESIAN")
depl:setMasterSlaveActivity("controlloop_scheduler","Robot_STATE_PUBLISHER")
depl:setMasterSlaveActivity("controlloop_scheduler","Robot_CMDDEMUX")

-- Creating connections policy
cp = rtt.Variable('ConnPolicy')
cp.type = rtt.globals.DATA   -- type data

-- Launching scheduler
print("Starting control loop")
controlloop_scheduler:configure()
controlloop_scheduler:start()

if run_status == SIM then	

	simulation_setup()
	robot_repub:configure()
	robot_cmddemux:configure()

	--visualization mode deactivated
	sim_visual_mode(2)

	rtt.logl('Info', "Robot SIM startup.")
	robot_sim:start()
	robot_republisher("SIM")
	robot_repub:start()

	rtt.logl('Info', "Robot KINE startup.")
	kinematic_setup()
	kinematic_to_robot("SIM")
	--kinematic_js_weight({1.0,1.0,1.0,1.0,1.0,1.0})
	kinematic_js_weight({1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0})
	rtt.logl('Info', "Cartesian CONTROLLER startup.")
	cartesian_controller_setup()

	--armSetCtrlModes(SIM,1,2)
	--cartesian_controller_start()
	vel_startup(SIM,1,1)

	rtt.logl('Info','Robot CMDDEMUX startup.')
	robot_cmddemux:start()

elseif run_status == HW then

	oodl_setup()
	robot_repub:configure()
	robot_cmddemux:configure()

	rtt.logl('Info', "Robot OODL start.")
	robot_oodl:start()
	robot_republisher("OODL")
	robot_repub:start()

	rtt.logl('Info', "Robot KINE startup.")
	kinematic_setup()
	kinematic_to_robot("OODL")
	kinematic_js_weight({1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0})	

	oodl_arm_op_clear[1]()
	oodl_base_op_clear[1]()

	rtt.logl('Info', "Robot CTRL CARTESIAN startup.")
	cartesian_controller_setup()	
	vel_startup(OODL,1,1)

	rtt.logl('Info','Robot CMDDEMUX startup.')
	robot_cmddemux:start()

elseif run_status == BOTH then

	simulation_setup()
	oodl_setup()
	robot_repub:configure()
	robot_cmddemux:configure()

	--visualization mode activated
	sim_visual_mode(1)

	queue_setup()	
	connect_command_from_ros(OODL)

	rtt.logl('Info', "Robot OODL start.")
	robot_oodl:start()
	robot_republisher("OODL")
	robot_repub:start()

	rtt.logl('Info', "Robot KINE startup.")
	kinematic_setup()
	kinematic_to_robot("OODL")
	kinematic_js_weight({1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0})	

	oodl_arm_op_clear[1]()
	oodl_base_op_clear[1]()

	rtt.logl('Info', "Robot CTRL CARTESIAN startup.")
	cartesian_controller_setup()	
	vel_startup(OODL,1,1)

	rtt.logl('Info','Robot CMDDEMUX startup.')
	robot_cmddemux:start()

end

if communication_type == REMOTE then

    local socket = require "socket"
    local udp = socket.udp()

    udp:settimeout(0)
    udp:setsockname(socket_address, socket_port)

    local world = {} -- the empty world-state
    local data, msg_or_ip, port_or_nil
    local entity, cmd, parms

    print "[REMOTE] Server loop started"

    while running do

       data, msg_or_ip, port_or_nil = udp:receivefrom()

		if data ~= nil then
			print(data)

			-- Experimental!-------------------------------------------------
			if data == "ROBOT_STATE" then

				r_state = rtt.Variable("sensor_msgs.JointState")
				fs, r_state = robot_repub:getPort("arm_state_in"):read()

				local count = 1
				local string = "JOINT_STATE"

				for i=1,5 do
					string = string..";"..r_state.position[i]
				end

				fs, r_state = robot_repub:getPort("base_state_in"):read()

				for i=1,4 do
					string = string..";"..r_state.position[i]
				end

				udp:sendto(string, msg_or_ip, socket_port)
			------------------------------------------------------------------
			else

				local status, err = pcall(function() loadstring(hash_remote_command[data])() end)

				if status then

					udp:sendto(data.." - ok.", msg_or_ip, socket_port)
					--print("[REMOTE] Reply sent "..msg_or_ip.." - "..socket_port)

				else

					print("[DEPLOYER] "..err)
					udp:sendto(data.." - failed.", msg_or_ip, socket_port)
					--print("[REMOTE] Failed call sent")

				end
			end
       	end
    end

elseif communication_type == LOCAL then

	while running do

		text = io.read("*l")

		if text == "move_up" then

			move(0,0,0.2)

		end

	end
end
