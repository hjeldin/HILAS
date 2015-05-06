SET(PUMA260_SIM_INCLUDE_DIR ${COMMON_INCLUDE_DIR})
SET(PUMA260_SIM_libs boost_thread)
SET(PUMA260_SIM_FILES Puma260GripperService.cpp Puma260ArmService.cpp puma260_SIM-component.cpp ${HILAS_INTERFACES}/IRobotArmService.cpp ${HILAS_INTERFACES}/IRobotGripperService.cpp ${HILAS_INTERFACES}/IRobot.cpp)

INCLUDE_DIRECTORIES(${PUMA260_SIM_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
#LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(Puma260_SIM ${PUMA260_SIM_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(Puma260_SIM ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${PUMA260_SIM_libs})
TARGET_LINK_LIBRARIES(Puma260_SIM RemoteApi)