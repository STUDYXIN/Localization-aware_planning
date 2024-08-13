cmake_minimum_required( VERSION 2.8.3)

project( normal_mpc )

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
add_compile_options(-std=c++14)

# Prerequisites
find_package( ACADO REQUIRED )

# Include directories
include_directories( . ${ACADO_INCLUDE_DIRS} )


# Build an executable
add_executable( normal_mpc_codegen normal_mpc.cpp Yaml.cpp)
target_link_libraries( normal_mpc_codegen ${ACADO_SHARED_LIBRARIES} )
set_target_properties( normal_mpc_codegen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
