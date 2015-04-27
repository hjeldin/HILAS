orocos_component(youbot_cmddemux ../include/youbot_cmddemux.hpp youbot_cmddemux.cpp ${HILAS_INTERFACES}/IRobotCmdDemux.cpp) 
# ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/youbot_kinematics/ directory
#
orocos_install_headers(youbot_cmddemux.hpp) # ...you may add multiple header files
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
#
#orocos_plugin(youbot_kinematics-plugin youbot_kinematics-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libyoubot_kinematics-service-<target>.so
# and installs in the directory lib/orocos/youbot_kinematics/plugins/
#
#0orocos_service(youbot_kinematics-service youbot_kinematics-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libyoubot_kinematics-types-<target>.so
# and installs in the directory lib/orocos/@target@/youbot_kinematics/types/
#
# The header will go in include/orocos/youbot_kinematics/types/youbot_kinematics/youbot_kinematics-types.hpp
# So you can #include <youbot_kinematics/youbot_kinematics-types.hpp>
#
#orocos_typegen_headers(include/youbot_kinematics/youbot_kinematics-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


