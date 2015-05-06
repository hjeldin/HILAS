orocos_component(puma260_republisher ../include/puma260_republisher-component.hpp puma260_republisher-component.cpp ${HILAS_INTERFACES}/IRobotStateRepublisher.cpp)
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/puma260_republisher/ directory
#
orocos_install_headers(puma260_republisher-component.hpp)