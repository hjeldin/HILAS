INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

orocos_component(TransformToTSim TransformToTSim.hpp TransformToTSim.cpp ${COMMON_INCLUDE_DIR}) 

orocos_install_headers( TransformToTSim.hpp )

