orocos_component(puma260_kinematics puma260_kinematics-component.hpp puma260_kinematics-component.cpp ${HILAS_INTERFACES}/IRobotKinematics.cpp) 
target_link_libraries(puma260_kinematics ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})

orocos_install_headers(puma260_kinematics-component.hpp) # ...you may add multiple header files