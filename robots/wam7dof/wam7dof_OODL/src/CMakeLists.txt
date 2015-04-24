SET(WAM8DOF_OODL_INCLUDE_DIR ${COMMON_INCLUDE_DIR})
SET(WAM8DOF_OODL_libs boost_thread)
SET(WAM8DOF_OODL_FILES Wam7dofGripperService.cpp Wam7dofArmService.cpp wam7dof_OODL-component.cpp ${HILAS_INTERFACES}/IRobotArmService.cpp ${HILAS_INTERFACES}/IRobotBaseService.cpp ${HILAS_INTERFACES}/IRobotGripperService.cpp ${HILAS_INTERFACES}/IRobot.cpp)

INCLUDE_DIRECTORIES(${WAM8DOF_OODL_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment

orocos_component(Wam7dof_OODL ${WAM8DOF_OODL_FILES} ${COMMON_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(Wam7dof_OODL ${WAM8DOF_OODL_libs} ${orocos_kdl_LIBRARIES})
TARGET_LINK_LIBRARIES(Wam7dof_OODL RemoteApi)