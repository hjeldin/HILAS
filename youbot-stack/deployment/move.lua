require("rttlib")
 
tc=rtt.getTC();
 
function configureHook()
   outport = rtt.OutputPort("/geometry_msgs/Pose", "moveOut")    -- global variable!
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
