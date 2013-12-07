SET(YOUBOT_VREP_INCLUDE_DIR
	${COMMON_INCLUDE_DIR}	
	${SOEM_INCLUDE_DIR}
	${VREP_YOUBOT_INCLUDE_DIR}
)

SET(YOUBOT_VREP_libs
	${SOEM_LIBRARIES}
	${VREP_YOUBOT_LIBRARIES}
	boost_thread
	YouBot_VREP_plugin
)

SET(YOUBOT_VREP_FILES YouBotGripperService.cpp YouBotArmService.cpp YouBotBaseService.cpp YouBotVREP.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_VREP_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(YouBot_VREP ${YOUBOT_VREP_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_VREP ${YOUBOT_VREP_libs})

set_target_properties(YouBot_VREP PROPERTIES COMPILE_FLAGS "-Wall")

orocos_install_headers( YouBotTypes.h )

#
# Plugin
#

#defines where static libraries are stored
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/plugins)

#defines where dynamic libraries (plugins) are stored
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin/plugins)

orocos_plugin(YouBot_VREP_plugin YouBotHelpersPlugin.cpp YouBotHelpers.cpp ${COMMON_INCLUDE_DIR})
