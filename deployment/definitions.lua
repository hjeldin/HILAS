-- Test functions defintion
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

	armSetCtrlModes(stype,3)

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

	armSetCtrlModes(stype,2)

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

	armSetCtrlModes(stype,1)

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

	baseSetCtrlModes(stype,5)

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

	baseSetCtrlModes(stype,3)

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

	baseSetCtrlModes(stype,2)

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

	baseSetCtrlModes(stype,1)

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

function vel_startup()

	armSetCtrlModes(OODL,2)
	baseSetCtrlModes(OODL,5)
	cartesian_controller_start()

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
  desiredPosPort:write(startPos)
  print("New setpoint EE")
end

function readSensPos()
  fs, pos = youbot_ctrl_cartesian:getPort("CartesianSensorPosition"):read()	
  print(pos)
end
