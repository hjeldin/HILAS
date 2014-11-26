SET(YOUBOT_OODL_INCLUDE_DIR ${OODL_YOUBOT_INCLUDE_DIR} ${COMMON_INCLUDE_DIR} ${SOEM_INCLUDE_DIR})
SET(YOUBOT_OODL_libs ${SOEM_LIBRARIES} ${OODL_YOUBOT_LIBRARIES} boost_thread)
SET(YOUBOT_OODL_FILES YouBotGripperService.cpp YouBotArmService.cpp YouBotBaseService.cpp YouBotOODL.cpp ${HILAS_INTERFACES}/IRobotArmService.cpp ${HILAS_INTERFACES}/IRobotBaseService.cpp ${HILAS_INTERFACES}/IRobotGripperService.cpp ${HILAS_INTERFACES}/IRobot.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_OODL_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

#Executable component and can also be a library usable for orocos deployment

orocos_component(YouBot_OODL ${YOUBOT_OODL_FILES} ${COMMON_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(YouBot_OODL ${YOUBOT_OODL_libs} ${orocos_kdl_LIBRARIES})
TARGET_LINK_LIBRARIES(YouBot_OODL RemoteApi_OODL)