SET(CMD_QUEUE_INCLUDE_DIR ${COMMON_INCLUDE_DIR})
SET(CMD_QUEUE_FILES Cmd_queue-component.cpp)
INCLUDE_DIRECTORIES(${CMD_QUEUE_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(Cmd_queue ${CMD_QUEUE_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(Cmd_queue ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})