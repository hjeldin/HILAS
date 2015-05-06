orocos_component(wam7dof_kinematics wam7dof_kinematics-component.hpp wam7dof_kinematics-component.cpp ${HILAS_INTERFACES}/IRobotKinematics.cpp) 
target_link_libraries(wam7dof_kinematics ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})

orocos_install_headers(wam7dof_kinematics-component.hpp) # ...you may add multiple header files