INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

orocos_component(YouBotStateRepublisher YouBotStateRepublisher.hpp YouBotStateRepublisher.cpp ${COMMON_INCLUDE_DIR}) 

orocos_install_headers( YouBotStateRepublisher.hpp ) 
