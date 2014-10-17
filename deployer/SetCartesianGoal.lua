require("rttlib")
 
tc=rtt.getTC();
 
function configureHook()
   outport = rtt.OutputPort("/geometry_msgs/Pose", "cartesianGoal_out")    -- global variable!
   tc:addPort(outport)
   return true
end
 
function updateHook()
	--Nothing
end
 
function cleanupHook()
   tc:removePort("outport")
   outport:delete()
end
