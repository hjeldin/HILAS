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
	kinematic_js_weight({1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0})

	rtt.logl('Info', "Cartesian CONTROLLER startup.")
	cartesian_controller_setup()

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