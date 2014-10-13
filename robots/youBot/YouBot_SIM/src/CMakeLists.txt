SET(YOUBOT_SIM_INCLUDE_DIR ${COMMON_INCLUDE_DIR} ${OODL_YOUBOT_INCLUDE_DIR} ${OODL_YOUBOT_INCLUDE_DIR}/include/youbot_driver)
SET(YOUBOT_SIM_libs ${OODL_YOUBOT_LIBRARIES} boost_thread YouBot_SIM_plugin)
SET(YOUBOT_SIM_FILES YouBotGripperService.cpp YouBotArmService.cpp YouBotBaseService.cpp YouBotSIM.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_SIM_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
#LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(YouBot_SIM ${YOUBOT_SIM_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_SIM ${orocos_kdl_LIBRARIES} ${YOUBOT_SIM_libs})
TARGET_LINK_LIBRARIES(YouBot_SIM RemoteApi)
#set_target_properties(YouBot_SIM PROPERTIES COMPILE_FLAGS "-Wall")

orocos_install_headers( YouBotTypes.h )

#
# Plugin
#

#defines where static libraries are stored
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/plugins)

#defines where dynamic libraries (plugins) are stored
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin/plugins)

orocos_plugin(YouBot_SIM_plugin YouBotHelpersPlugin.cpp YouBotHelpers.cpp ${COMMON_INCLUDE_DIR})
