SET(YOUBOT_SIM_INCLUDE_DIR ${COMMON_INCLUDE_DIR} ${OODL_YOUBOT_INCLUDE_DIR})
SET(YOUBOT_SIM_libs ${OODL_YOUBOT_LIBRARIES} boost_thread)
SET(YOUBOT_SIM_FILES YouBotGripperService.cpp YouBotArmService.cpp YouBotBaseService.cpp YouBotSIM.cpp ${HILAS_INTERFACES}/IRobotArmService.cpp ${HILAS_INTERFACES}/IRobotBaseService.cpp ${HILAS_INTERFACES}/IRobotGripperService.cpp ${HILAS_INTERFACES}/IRobot.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_SIM_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
#LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(YouBot_SIM ${YOUBOT_SIM_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_SIM ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${YOUBOT_SIM_libs})
TARGET_LINK_LIBRARIES(YouBot_SIM RemoteApi)