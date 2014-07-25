SET(YOUBOT_EXECUTIVE_INCLUDE_DIR ${COMMON_INCLUDE_DIR} )

SET(YOUBOT_EXECUTIVE_FILES YouBot_executive.cpp ExecutiveHelpers.cpp)
INCLUDE_DIRECTORIES(${YOUBOT_EXECUTIVE_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
#orocos_component(YouBot_EXECUTIVE ${YOUBOT_EXECUTIVE_FILES} ${COMMON_INCLUDE_DIR})
orocos_component(YouBot_executive ${YOUBOT_EXECUTIVE_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_executive ${YOUBOT_EXECUTIVE_libs})

set_target_properties(YouBot_executive PROPERTIES COMPILE_FLAGS "-Wall")