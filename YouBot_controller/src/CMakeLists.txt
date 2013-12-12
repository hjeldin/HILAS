SET(YOUBOT_CONTROLLER_INCLUDE_DIR ${COMMON_INCLUDE_DIR} )

SET(YOUBOT_CONTROLLER_FILES YouBot_controller-component.cpp)
INCLUDE_DIRECTORIES(${YOUBOT_CONTROLLER_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(YouBot_controller ${YOUBOT_CONTROLLER_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_controller ${YOUBOT_CONTROLLER_libs})

set_target_properties(YouBot_controller PROPERTIES COMPILE_FLAGS "-Wall")