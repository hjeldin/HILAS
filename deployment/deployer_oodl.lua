require "rttlib"
require "rfsm_rtt"
require "rttros"
require "kdlpp"
require "kdlutils"

require "complete"
require "readline"
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
depl:import("kdl_typekit")
depl:import("rtt_sensor_msgs")
depl:import("rtt_std_msgs")
depl:import("rtt_geometry_msgs")
depl:import("rtt_nav_msgs")
depl:import("rtt_motion_control_msgs")
depl:import("rtt_rosnode")

depl:import("YouBot_VREP")
depl:import("YouBot_OODL")
depl:import("YouBot_queue")

depl:import("youbot_kinematics")
depl:import("cartesian_motion_control")

-- Loading component
depl:loadComponent("controlloop_scheduler", "FBSched")
depl:loadComponent("YouBot_OODL", "YouBot::YouBotOODL")
depl:loadComponent("YouBot_VREP", "YouBot::YouBotVREP") 
depl:loadComponent("YouBot_QUEUE", "YouBot_queue")

depl:loadComponent("YouBot_KINE", "Youbot_kinematics")	
depl:loadComponent("YouBot_CTRL_CARTESIAN", "MotionControl::CartesianControllerPos")

depl:loadComponent("VREP_VISMODE", "OCL::LuaComponent")
depl:loadComponent("MOVE_OUT", "OCL::LuaComponent")

-- Getting peers of components
controlloop_scheduler = depl:getPeer("controlloop_scheduler")
youbot_vrep = depl:getPeer("YouBot_VREP")
youbot_oodl = depl:getPeer("YouBot_OODL")
youbot_queue = depl:getPeer("YouBot_QUEUE")

youbot_kine = depl:getPeer("YouBot_KINE")
youbot_ctrl_cartesian = depl:getPeer("YouBot_CTRL_CARTESIAN")

vrep_vismode = depl:getPeer("VREP_VISMODE")
move_out = depl:getPeer("MOVE_OUT")

-- Using fbsched for activity
depl:setActivity("controlloop_scheduler",0.002,99,rtt.globals.ORO_SCHED_RT)
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_VREP")
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_OODL")
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_QUEUE")

depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_KINE")
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_CTRL_CARTESIAN")

vrep_vismode:exec_file("visualMode.lua")
move_out:exec_file("move.lua")

-- Creating connections policy
cp = rtt.Variable('ConnPolicy')
cp.type = rtt.globals.DATA   -- type data

-- Launching scheduler
print("Starting control loop")
controlloop_scheduler:configure()
controlloop_scheduler:start()

vrep_vismode:configure()
vrep_vismode:start()

move_out:configure()
move_out:start()

depl:stream("VREP_VISMODE.visMode", rtt.provides("ros"):topic("/vrep/vis_mode"))

if run_status == SIM then
	--visualization mode deactivated
	vrep_visual_mode(0)
	
	simulation_setup()
	connect_vrep_ros_streams()

	cartesian_controller_setup()
	cartesian_input_from_vrep()

	rtt.logl('Info', "Youbot VREP start.")
	youbot_vrep:start()

	rtt.logl('Info', "Youbot CTRL CARTESIAN start.")
	cartesian_controller_start()

elseif run_status == HW then
	--visualization mode activated
	vrep_visual_mode(1)

	oodl_setup()

	cartesian_controller_setup()
	cartesian_input_from_oodl()

	rtt.logl('Info', "Youbot OODL start.")
	youbot_oodl:start()
	oodl_arm_op_clear()
	oodl_base_op_clear()

	rtt.logl('Info', "Youbot CTRL CARTESIAN start.")
	cartesian_controller_start()

elseif run_status == BOTH then
	--visualization mode activated
	vrep_visual_mode(1)

	simulation_setup()
	oodl_setup()
	queue_setup()
	cartesian_controller_setup()
	cartesian_input_from_oodl()

	connect_oodl_ros_streams()

	rtt.logl('Info', "Youbot OODL start.")
	youbot_oodl:start()
	oodl_arm_op_clear()
	oodl_base_op_clear()

	rtt.logl('Info', "Youbot VREP start.")
	youbot_vrep:start()

	rtt.logl('Info', "Youbot CTRL CARTESIAN start.")
	--cartesian_controller_start()

end

if communication_type == REMOTE then

    local socket = require "socket"
    local udp = socket.udp()

    udp:settimeout(0)
    udp:setsockname('*', 22223)

    local world = {} -- the empty world-state
    local data, msg_or_ip, port_or_nil
    local entity, cmd, parms

    print "Beginning server loop."

    while running do

        data, msg_or_ip, port_or_nil = udp:receivefrom()

        if data == "move_up" then

			move(0,0,0.2)

        elseif msg_or_ip ~= 'timeout' then

            error("Unknown network error: "..tostring(msg))

        end

    end

elseif communication_type == LOCAL then

	while running do

		text = io.read("*l")

		if text == "move_up" then

			move(0,0,0.2)

		end

	end

elseif communication_type == DEBUG then

end
