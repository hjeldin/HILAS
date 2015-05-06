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

	kinematic_js_weight({1.0,1.0,1.0,1.0,1.0,1.0,1.0})
	rtt.logl('Info', "Cartesian CONTROLLER startup.")
	cartesian_controller_setup()

	armSetCtrlModes(SIM,1,2)
	cartesian_controller_start()

	rtt.logl('Info','Robot CMDDEMUX startup.')
	robot_cmddemux:start()

elseif run_status == HW then

	print "not implemented"

elseif run_status == BOTH then

	print "not implemented"

end