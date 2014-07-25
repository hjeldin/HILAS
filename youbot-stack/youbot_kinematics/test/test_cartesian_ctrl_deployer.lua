require "rttlib"
require "rttros"

--ya_file = rttros.find_rospack("youbot_driver_rtt") .. "/lua/youbot_test.lua"
--dofile(ya_file)

--tc=rtt.getTC()
--depl=tc:getPeer("Deployer")

------------------------------------------------------------------------
-- Lua deploy
------------------------------------------------------------------------

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

----

--depl:import("youbot_kinematics")
depl:import("cartesian_motion_control")
depl:import("kdl_typekit")
--depl:loadComponent("kine", "Youbot_kinematics")	
depl:loadComponent("controller", "MotionControl::CartesianControllerPos")
--kine=depl:getPeer("kine")
--depl:loadService("kine","rosparam")
--kine:provides("rosparam"):refreshProperty("robot_description",false,false)
--kine=depl:getPeer("kine")
control=depl:getPeer("controller")
K = control:getProperty("K")
local gain = 1.0
K:fromtab{gain,gain,gain,gain,gain,gain}
print(K)
--kine:configure()
control:configure()
--kine:start()
control:start()

-- depl:connect("youbot.Arm1.jointstate","kine.JointState",rtt.Variable("ConnPolicy"))
-- depl:connect("youbot.Arm1.joint_velocity_command","kine.JointVelocities",rtt.Variable("ConnPolicy"))
-- depl:connect("youbot.Base.cmd_twist","kine.BaseTwist",rtt.Variable("ConnPolicy"))
-- depl:connect("kine.EEPose","controller.CartesianSensorPosition",rtt.Variable("ConnPolicy"))
-- depl:connect("kine.EETwist","controller.CartesianOutputVelocity",rtt.Variable("ConnPolicy"))
-- kine=depl:getPeer("kine")
-- control=depl:getPeer("controller")
-- K = control:getProperty("K")
-- local gain = 1.0
-- K:fromtab{gain,gain,gain,gain,gain,gain}
-- print(K)
-- kine:configure()
-- control:configure()
-- kine:start()
-- control:start()

-- desi = rttlib.port_clone_conn(control:getPort("CartesianDesiredPosition"))
-- js_weight_port = rttlib.port_clone_conn(kine:getPort("JointSpaceWeights"))
-- js_weight = rtt.Variable("float64[]")
-- js_weight:resize(8)
-- js_weight:fromtab{1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0}
-- js_weight_port:write(js_weight)
-- fs, pos = control:getPort("CartesianSensorPosition"):read()
-- desi:write(pos)

-- cp = rtt.Variable("ConnPolicy")
-- cp.transport=3
-- cp.name_id= "/youbot/ee"
-- depl:stream("kine.EEPose",cp)

-- cp.name_id="/youbot/desired_ee"
-- depl:stream("controller.CartesianDesiredPosition",cp)
