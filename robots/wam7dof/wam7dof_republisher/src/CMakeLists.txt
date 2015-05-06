orocos_component(wam7dof_republisher ../include/wam7dof_republisher-component.hpp wam7dof_republisher-component.cpp ${HILAS_INTERFACES}/IRobotStateRepublisher.cpp)
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/wam7dof_republisher/ directory
#
orocos_install_headers(wam7dof_republisher-component.hpp)