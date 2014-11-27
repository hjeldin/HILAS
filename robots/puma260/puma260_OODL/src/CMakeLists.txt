SET(PUMA260_OODL_INCLUDE_DIR ${COMMON_INCLUDE_DIR})
SET(PUMA260_OODL_libs boost_thread)
SET(PUMA260_OODL_FILES Puma260GripperService.cpp Puma260ArmService.cpp puma260_OODL-component.cpp ${HILAS_INTERFACES}/IRobotArmService.cpp ${HILAS_INTERFACES}/IRobotBaseService.cpp ${HILAS_INTERFACES}/IRobotGripperService.cpp ${HILAS_INTERFACES}/IRobot.cpp)

INCLUDE_DIRECTORIES(${PUMA260_OODL_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment

orocos_component(Puma260_OODL ${PUMA260_OODL_FILES} ${COMMON_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(Puma260_OODL ${PUMA260_OODL_libs} ${orocos_kdl_LIBRARIES})
TARGET_LINK_LIBRARIES(Puma260_OODL RemoteApi)