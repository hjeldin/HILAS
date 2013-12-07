require "rttlib"
require "rfsm_rtt"
require "rttros"
require "kdlpp"
require "kdlutils"

--ya_file = rttros.find_rospack("youbot_driver_rtt") .. "/lua/youbot_test.lua"
--dofile(ya_file)

tc=rtt.getTC()
depl=tc:getPeer("Deployer")

depl:import("youbot_kinematics")
depl:import("cartesian_motion_control")
depl:import("kdl_typekit")
depl:loadComponent("kine", "Youbot_kinematics")	
depl:loadComponent("controller", "MotionControl::CartesianControllerPos")
--depl:connect("youbot.Arm1.jointstate","kine.JointState",rtt.Variable("ConnPolicy"))
--depl:connect("youbot.Arm1.joint_velocity_command","kine.JointVelocities",rtt.Variable("ConnPolicy"))
--depl:connect("youbot.Base.cmd_twist","kine.BaseTwist",rtt.Variable("ConnPolicy"))
--depl:connect("kine.EEPoseRTT","controller.CartesianSensorPosition",rtt.Variable("ConnPolicy"))
depl:connect("kine.EEPose","controller.CartesianSensorPosition",rtt.Variable("ConnPolicy"))
depl:connect("kine.EETwistRTT","controller.CartesianOutputVelocity",rtt.Variable("ConnPolicy"))
kine=depl:getPeer("kine")
depl:loadService("kine","rosparam")
kine:provides("rosparam"):refreshProperty("robot_description",false,false)
kine=depl:getPeer("kine")
control=depl:getPeer("controller")
K = control:getProperty("K")
local gain = 0.1
K:fromtab{gain,gain,gain,gain,gain,gain}
print(K)

cp = rtt.Variable("ConnPolicy")
cp.transport=3
cp.name_id= "/youbot/EEPose"
depl:stream("kine.EEPose",cp)

cp.name_id="/youbot/desired_ee"
depl:stream("controller.CartesianDesiredPosition",cp)

cp.name_id="/joint_states"
depl:stream("kine.JointState",cp)

cp.name_id="/arm_1/arm_controller/velocity_command"
--cp.name_id="/velocity_command"
depl:stream("kine.JointVelocities",cp)

cp.name_id="/cmd_vel"
depl:stream("kine.BaseTwist",cp)

cp.name_id="/odom"
depl:stream("kine.BaseOdom",cp)

print("ROS topic enable")

control:setPeriod(0.005)
kine:setPeriod(0.005)

kine:configure()
control:configure()
kine:start()

desiredPosPort = rttlib.port_clone_conn(control:getPort("CartesianDesiredPosition"))
print("Configure JointSpaceWeights")
js_weight_port = rttlib.port_clone_conn(kine:getPort("JointSpaceWeights"))
js_weight = rtt.Variable("float64[]")
js_weight:resize(8)
js_weight:fromtab{1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0}
js_weight_port:write(js_weight)
fs, startPos = control:getPort("CartesianSensorPosition"):read()
desiredPosPort:write(startPos)
print("Init Cartesian Controller")
control:start()

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
