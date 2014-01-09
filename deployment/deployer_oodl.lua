require "rttlib"
require "rfsm_rtt"
require "rttros"
require "kdlpp"
require "kdlutils"

require "complete"
require "readline"

--coroutine.create(udp_server)
rttlib.color = true

-- Enum definition
SIM, HW, BOTH, LUA_DEPLOYER, OPS_DEPLOYER, VREP, OODL = 0, 1, 2, 3, 4, 5 ,6

-- Deployer setup
run_status = BOTH
deployer_type = LUA_DEPLOYER

TOPIC_ARM_POSITION_COMMAND = "/arm_1/arm_controller/position_command"
TOPIC_ARM_VELOCITY_COMMAND = "/arm_1/arm_controller/velocity_command"
TOPIC_ARM_EFFORT_COMMAND = "/arm_1/arm_controller/torque_command"
TOPIC_BASE_TWIST_COMMAND = "/cmd_vel"
TOPIC_BASE_ODOM_STATE = "/odom"
TOPIC_GRIPPER_POSITION_COMMAND = "/arm_1/gripper_controller/position_command"
VREP_TOPIC_ARM_JOINT_STATES_RX = "/vrep/hw_rx/arm_1/joint_state"
VREP_TOPIC_BASE_JOINT_STATES_RX = "/vrep/hw_rx/base/joint_state"
VREP_TOPIC_ODOM_STATE_RX = "/vrep/hw_rx/odom"

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

-- Getting peers of components
controlloop_scheduler = depl:getPeer("controlloop_scheduler")
youbot_vrep = depl:getPeer("YouBot_VREP")
youbot_oodl = depl:getPeer("YouBot_OODL")
youbot_queue = depl:getPeer("YouBot_QUEUE")

youbot_kine = depl:getPeer("YouBot_KINE")
youbot_ctrl_cartesian = depl:getPeer("YouBot_CTRL_CARTESIAN")

vrep_vismode = depl:getPeer("VREP_VISMODE")

-- Using fbsched for activity
depl:setActivity("controlloop_scheduler",0.002,99,rtt.globals.ORO_SCHED_RT)
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_VREP")
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_OODL")
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_QUEUE")

depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_KINE")
depl:setMasterSlaveActivity("controlloop_scheduler","YouBot_CTRL_CARTESIAN")

vrep_vismode:exec_file("visualMode.lua")

-- Creating connections policy
cp = rtt.Variable('ConnPolicy')
cp.type = rtt.globals.DATA   -- type data

-- Launching scheduler
print("Starting control loop")
controlloop_scheduler:configure()
controlloop_scheduler:start()

vrep_vismode:configure()
vrep_vismode:start()

depl:stream("VREP_VISMODE.visMode", rtt.provides("ros"):topic("/vrep/vis_mode"))

function vrep_visual_mode(a)

	visModePort = vrep_vismode:getPort("visMode")
	x = rtt.Variable("/std_msgs/Int32")
	x.data = a
	visModePort:write(x)

end

-- Definition setup functions
function simulation_setup()

	rtt.logl('Info', "Youbot VREP configure.")
	youbot_vrep:configure()

	vrep_arm_serv = youbot_vrep:provides("Arm1")
	vrep_base_serv = youbot_vrep:provides("Base")
	vrep_grip_serv = youbot_vrep:provides("Gripper1")

	vrep_arm_op_clear = vrep_arm_serv:getOperation("clearControllerTimeouts")
	vrep_arm_op_stat = vrep_arm_serv:getOperation("displayMotorStatuses")

	vrep_base_op_clear = vrep_base_serv:getOperation("clearControllerTimeouts")
	vrep_base_op_stat = vrep_base_serv:getOperation("displayMotorStatuses")

	require "definitions"

end

function queue_setup()

	youbot_queue:configure()
	youbot_queue:start()
	queue_op_is_loading = youbot_queue:getOperation("setIsInLoading")

end

function queue_input_disconnect()

	youbot_queue:getPort("out_arm_joint_position_command"):disconnect()
	youbot_queue:getPort("out_arm_joint_velocity_command"):disconnect()
	youbot_queue:getPort("out_arm_joint_effort_command"):disconnect()
	youbot_queue:getPort("out_base_cmd_twist"):disconnect()
	youbot_queue:getPort("out_gripper_joint_position_command"):disconnect()
	youbot_queue:getPort("out_ros_planner_command"):disconnect()
	youbot_queue:getPort("out_ros_cartesian_command"):disconnect()

end

function queue_input_connect()

	depl:stream("YouBot_QUEUE.ros_arm_joint_position_command", rtt.provides("ros"):topic(TOPIC_ARM_POSITION_COMMAND))
	depl:stream("YouBot_QUEUE.ros_arm_joint_velocity_command", rtt.provides("ros"):topic(TOPIC_ARM_VELOCITY_COMMAND))
	depl:stream("YouBot_QUEUE.ros_arm_joint_effort_command", rtt.provides("ros"):topic(TOPIC_ARM_EFFORT_COMMAND))
	depl:stream("YouBot_QUEUE.ros_base_cmd_twist", rtt.provides("ros"):topic(TOPIC_BASE_TWIST_COMMAND))
	depl:stream("YouBot_QUEUE.ros_gripper_joint_position_command", rtt.provides("ros"):topic(TOPIC_GRIPPER_POSITION_COMMAND))
	depl:stream("YouBot_QUEUE.ros_planner_command", rtt.provides("ros"):topic("/move_base_simple/goal"))
	depl:stream("YouBot_QUEUE.ros_cartesian_command", rtt.provides("ros"):topic("/youbot/desired_ee"))

end

function queue_output_disconnect()

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

end

function cartesian_controller_setup()

	depl:connect("YouBot_KINE.EEPose","YouBot_CTRL_CARTESIAN.CartesianSensorPosition",cp)
	depl:connect("YouBot_KINE.EETwistRTT","YouBot_CTRL_CARTESIAN.CartesianOutputVelocity",cp)

	depl:loadService("YouBot_KINE","rosparam")

	youbot_kine:provides("rosparam"):refreshProperty("robot_description",false,false)
	K = youbot_ctrl_cartesian:getProperty("K")
	local gain = 0.1
	K:fromtab{gain,gain,gain,gain,gain,gain}

	depl:stream("YouBot_KINE.EEPose",rtt.provides("ros"):topic("/youbot/EEPose"))
	cartesian_goal_connect()
end

function cartesian_controller_start()

	youbot_kine:configure()
	youbot_ctrl_cartesian:configure()
	youbot_kine:start()
	youbot_ctrl_cartesian:start()

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

	depl:stream("YouBot_CTRL_CARTESIAN.CartesianDesiredPosition",rtt.provides("ros"):topic("/youbot/desired_ee"))

end

function cartesian_goal_disconnect()

	youbot_ctrl_cartesian:getPort("CartesianDesiredPosition"):disconnect()

end


function cartesian_input_from_vrep()

	depl:stream("YouBot_KINE.JointState",rtt.provides("ros"):topic("/joint_states"))
	depl:stream("YouBot_KINE.JointVelocities",rtt.provides("ros"):topic("/arm_1/arm_controller/velocity_command"))
	depl:stream("YouBot_KINE.BaseTwist",rtt.provides("ros"):topic("/cmd_vel"))
	depl:stream("YouBot_KINE.BaseOdom",rtt.provides("ros"):topic("/odom"))

end

function cartesian_input_from_oodl()

	depl:connect("YouBot_KINE.JointState","YouBot_OODL.Arm1.joint_state",cp)
	depl:connect("YouBot_KINE.JointVelocities","YouBot_OODL.Arm1.joint_velocity_command",cp)
	depl:connect("YouBot_KINE.BaseTwist","YouBot_OODL.Base.cmd_twist",cp)
	depl:connect("YouBot_KINE.BaseOdom","YouBot_OODL.Base.odometry_state",cp)

end

function connect_vrep_ros_streams()
	-- ROS simulated robot streaming
	depl:stream("YouBot_VREP.Arm1.in_joint_state",rtt.provides("ros"):topic("/vrep/arm_1/joint_states"))
	depl:stream("YouBot_VREP.Base.in_joint_state",rtt.provides("ros"):topic("/vrep/base/joint_states"))
	depl:stream("YouBot_VREP.Base.in_odometry_state",rtt.provides("ros"):topic("/odom"))

	depl:stream("YouBot_VREP.Arm1.out_joint_position_command",rtt.provides("ros"):topic(TOPIC_ARM_POSITION_COMMAND))
	depl:stream("YouBot_VREP.Arm1.out_joint_velocity_command",rtt.provides("ros"):topic(TOPIC_ARM_VELOCITY_COMMAND))
	depl:stream("YouBot_VREP.Arm1.out_joint_effort_command",rtt.provides("ros"):topic(TOPIC_ARM_EFFORT_COMMAND))
	depl:stream("YouBot_VREP.Base.out_joint_position_command",rtt.provides("ros"):topic("/base/base_controller/position_command"))
	depl:stream("YouBot_VREP.Base.out_joint_velocity_command",rtt.provides("ros"):topic("/base/base_controller/velocity_command"))
	depl:stream("YouBot_VREP.Base.out_joint_effort_command",rtt.provides("ros"):topic("/base/base_controller/force_command"))
	depl:stream("YouBot_VREP.Base.out_cmd_twist",rtt.provides("ros"):topic(TOPIC_BASE_TWIST_COMMAND))
	depl:stream("YouBot_VREP.Gripper1.out_gripper_cmd_position",rtt.provides("ros"):topic(TOPIC_GRIPPER_POSITION_COMMAND))
end

function disconnect_vrep_ros_streams() 
	--ARM 
	vrep_arm_serv:getPort("in_joint_state"):disconnect()
	vrep_arm_serv:getPort("out_joint_position_command"):disconnect()
	vrep_arm_serv:getPort("out_joint_velocity_command"):disconnect()
	vrep_arm_serv:getPort("out_joint_effort_command"):disconnect()
	--BASE
	vrep_base_serv:getPort("out_joint_position_command"):disconnect()
	vrep_base_serv:getPort("out_joint_velocity_command"):disconnect()
	vrep_base_serv:getPort("out_joint_effort_command"):disconnect()
	vrep_base_serv:getPort("out_cmd_twist"):disconnect()
	vrep_base_serv:getPort("in_joint_state"):disconnect()
	vrep_base_serv:getPort("in_odometry_state"):disconnect()
	--GRIPPER
	vrep_grip_serv:getPort("out_gripper_cmd_position"):disconnect()
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

	require "definitions"

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

	--VREP streams (VISUALIZATION_MODE)
    depl:stream("YouBot_OODL.Base.odometry_state",rtt.provides("ros"):topic(VREP_TOPIC_ODOM_STATE_RX))
    depl:stream("YouBot_OODL.Base.joint_state",rtt.provides("ros"):topic(VREP_TOPIC_BASE_JOINT_STATES_RX))
    depl:stream("YouBot_OODL.Arm1.joint_state",rtt.provides("ros"):topic(VREP_TOPIC_ARM_JOINT_STATES_RX))

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

function switch_to(mode)

	if mode == SIM then
		-- visualization mode deactivated
		vrep_visual_mode(0)
		--disconnect HW ports and streams
		disconnect_oodl_ros_streams()
		--connect SIM ports and streams
		connect_vrep_ros_streams()
		--connect queue ports and streams
		queue_output_disconnect()
		queue_input_connect()
		--connect cartesian ports and streams
		cartesian_input_from_vrep()
		--activate queue recording
		queue_op_is_loading:send(true)

	elseif mode == HW then
		-- visualization mode activated
		vrep_visual_mode(1)
		--disconnect SIM ports and streams
		disconnect_vrep_ros_streams()
		--connect HW ports and streams
		connect_oodl_ros_streams()
		--connect queue ports and streams
		queue_input_disconnect()
		cartesian_goal_disconnect()
		queue_output_connect()
		--connect cartesian ports and streams
		cartesian_input_from_oodl()
		--activate queue downloading
		queue_op_is_loading:send(false)
	end
end

-- Mode setup
if run_status == SIM then
	-- visualization mode deactivated
	vrep_visual_mode(0)
	
	simulation_setup()
	cartesian_controller_setup()
	cartesian_input_from_vrep()

	rtt.logl('Info', "Youbot VREP start.")
	youbot_vrep:start()

	rtt.logl('Info', "Youbot CTRL CARTESIAN start.")
	cartesian_controller_start()

elseif run_status == HW then
	-- visualization mode activated
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
	-- visualization mode activated
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
