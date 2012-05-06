INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

orocos_component(TSim_to_YouBot TSim_to_YouBot.hpp TSim_to_YouBot.cpp ${COMMON_INCLUDE_DIR}) 

orocos_install_headers( TSim_to_YouBot.hpp )
