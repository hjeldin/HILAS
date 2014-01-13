require("rttlib")
 
tc=rtt.getTC();
 
function configureHook()
   outport = rtt.OutputPort("/std_msgs/Int32", "visMode")    -- global variable!
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