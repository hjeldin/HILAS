require "rttlib"
require "rfsm_rtt"
require "rttros"
require "kdlpp"
require "kdlutils"
require "complete"
require "readline"

--ya_file = rttros.find_rospack("youbot_driver_rtt") .. "/lua/youbot_test.lua"
--dofile(ya_file)

tc=rtt.getTC()
depl=tc:getPeer("Deployer")

depl:import("youbot_kinematics")
depl:import("cartesian_motion_control")
depl:import("kdl_typekit")
depl:loadComponent("kine", "Youbot_kinematics")	
depl:loadComponent("controller", "MotionControl::CartesianControllerPos")
-- GeneratorPos
depl:loadComponent("generatorPos", "MotionControl::CartesianGeneratorPos")

--depl:connect("youbot.Arm1.jointstate","kine.JointState",rtt.Variable("ConnPolicy"))
--depl:connect("youbot.Arm1.joint_velocity_command","kine.JointVelocities",rtt.Variable("ConnPolicy"))
--depl:connect("youbot.Base.cmd_twist","kine.BaseTwist",rtt.Variable("ConnPolicy"))
--depl:connect("kine.EEPoseRTT","controller.CartesianSensorPosition",rtt.Variable("ConnPolicy"))
depl:connect("kine.EEPose","controller.CartesianSensorPosition",rtt.Variable("ConnPolicy"))
depl:connect("kine.EETwistRTT","controller.CartesianOutputVelocity",rtt.Variable("ConnPolicy"))

-- Generator Pos
depl:connect("kine.EEPose","generatorPos.CartesianPoseMsr",rtt.Variable("ConnPolicy"))
depl:connect("controller.CartesianDesiredPosition", "generatorPos.CartesianPoseDes",rtt.Variable("ConnPolicy"))

kine=depl:getPeer("kine")
depl:loadService("kine","rosparam")
kine:provides("rosparam"):refreshProperty("robot_description",false,false)
kine=depl:getPeer("kine")
control=depl:getPeer("controller")
K = control:getProperty("K")
local gain = 0.02
K:fromtab{gain,gain,gain,gain,gain,gain}
print(K)

genPos=depl:getPeer("generatorPos")

cp = rtt.Variable("ConnPolicy")
cp.type = 2
cp.size = 100
cp.init = true
cp.lock_policy = 0
cp.transport=3
cp.name_id= "/youbot/EEPose"
depl:stream("kine.EEPose",cp)

--INTERACTIVE MARKER on RVIZ
--cp.name_id="/interactiveEEPose"
--depl:stream("generatorPos.CartesianPoseGoal",cp)

cp.name_id="/test_genPos"
depl:stream("generatorPos.CartesianPoseDes",cp)

cp.name_id="/vrep/arm_1/joint_states"
depl:stream("kine.JointState",cp)

cp.name_id="/arm_1/arm_controller/velocity_command"
--cp.name_id="/velocity_command"
depl:stream("kine.JointVelocities",cp)

cp.name_id="/cmd_vel"
depl:stream("kine.BaseTwist",cp)

cp.name_id="/odom"
depl:stream("kine.BaseOdom",cp)

print("ROS topic enable")

genPos:setPeriod(0.002)
control:setPeriod(0.002)
kine:setPeriod(0.002)

kine:configure()
control:configure()

-- max_vel=genPos:getProperty("max_vel")
-- max_vel.linear.x=1
-- max_vel.linear.y=1
-- max_vel.linear.z=1

-- max_vel.angular.x=1
-- max_vel.angular.y=1
-- max_vel.angular.z=1

-- max_vel=genPos:getProperty("max_acc")
-- max_vel.linear.x=1
-- max_vel.linear.y=1
-- max_vel.linear.z=1

-- max_vel.angular.x=1
-- max_vel.angular.y=1
-- max_vel.angular.z=1

genPos:configure()

kine:start()
control:start()
genPos:start()

fs,pos = genPos:getPort("CartesianPoseMsr"):read()

desiredPosPort = rttlib.port_clone_conn(control:getPort("CartesianDesiredPosition"))
print("Configure JointSpaceWeights")
js_weight_port = rttlib.port_clone_conn(kine:getPort("JointSpaceWeights"))
js_weight = rtt.Variable("float64[]")
js_weight:resize(8)
js_weight:fromtab{0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0}
js_weight_port:write(js_weight)
fs, startPos = control:getPort("CartesianSensorPosition"):read()
print("Start Position of Controller")
print(startPos)
desiredPosPort:write(startPos)
print("Init Cartesian Controller")
--control:start()

function setK(gain)
  control:stop()
  K:fromtab{gain,gain,gain,gain,gain,gain}
  control:configure()
  control:start()
  print(K)
end

function move(dx, dy, dz)
  fs, startPos = control:getPort("CartesianSensorPosition"):read()
  startPos.position.x = startPos.position.x + dx
  startPos.position.y = startPos.position.y + dy
  startPos.position.z = startPos.position.z + dz
  -- startPos.orientation.x = 0.7071067811865476;
  -- startPos.orientation.y = 0;
  -- startPos.orientation.z = 0;
  -- startPos.orientation.w = -0.7071067811865476;
  desiredPosPort:write(startPos)
  print("New setpoint EE")
end

function readSensPos()
  fs, pos = control:getPort("CartesianSensorPosition"):read()	
  print(pos)
end