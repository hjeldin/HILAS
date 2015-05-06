SET(Wam7dof_SIM_INCLUDE_DIR ${COMMON_INCLUDE_DIR})
SET(Wam7dof_SIM_libs boost_thread)
SET(Wam7dof_SIM_FILES Wam7dofGripperService.cpp Wam7dofArmService.cpp wam7dof_SIM-component.cpp ${HILAS_INTERFACES}/IRobotArmService.cpp ${HILAS_INTERFACES}/IRobotGripperService.cpp ${HILAS_INTERFACES}/IRobot.cpp)

INCLUDE_DIRECTORIES(${Wam7dof_SIM_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
#LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(Wam7dof_SIM ${Wam7dof_SIM_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(Wam7dof_SIM ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${Wam7dof_SIM_libs})
TARGET_LINK_LIBRARIES(Wam7dof_SIM RemoteApi)