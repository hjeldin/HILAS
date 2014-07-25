SET(YOUBOT_QUEUE_INCLUDE_DIR ${COMMON_INCLUDE_DIR} )

SET(YOUBOT_QUEUE_FILES YouBot_queue-component.cpp)
INCLUDE_DIRECTORIES(${YOUBOT_QUEUE_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(YouBot_queue ${YOUBOT_QUEUE_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_queue ${YOUBOT_QUEUE_libs})

set_target_properties(YouBot_queue PROPERTIES COMPILE_FLAGS "-Wall")