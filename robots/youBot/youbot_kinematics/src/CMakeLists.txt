orocos_component(youbot_kinematics youbot_kinematics-component.hpp youbot_kinematics-component.cpp ${HILAS_INTERFACES}/IRobotKinematics.cpp) 
target_link_libraries(youbot_kinematics ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES} ${OODL_YOUBOT_LIBRARIES})
# ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/youbot_kinematics/ directory
#
orocos_install_headers(youbot_kinematics-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libyoubot_kinematics-plugin-<target>.so
# and installs in the directory lib/orocos/youbot_kinematics/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.