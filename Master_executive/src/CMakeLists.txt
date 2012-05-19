SET(MASTER_EXECUTIVE_INCLUDE_DIR ${COMMON_INCLUDE_DIR} )

SET(MASTER_EXECUTIVE_FILES Master_executive.cpp)

INCLUDE_DIRECTORIES(${MASTER_EXECUTIVE_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
#orocos_component(YouBot_EXECUTIVE ${MASTER_EXECUTIVE_FILES} ${COMMON_INCLUDE_DIR})
orocos_component(Master_executive ${MASTER_EXECUTIVE_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(Master_executive ${MASTER_EXECUTIVE_libs})

set_target_properties(Master_executive PROPERTIES COMPILE_FLAGS "-Wall")

