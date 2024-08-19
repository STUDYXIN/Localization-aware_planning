# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( nmpc_model CXX)

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )


##################  用于nmpc求解   #############################
# cs_add_library(mpc_solver
#   externals/qpoases/SRC/Bounds.cpp
#   externals/qpoases/SRC/Constraints.cpp
#   externals/qpoases/SRC/CyclingManager.cpp
#   externals/qpoases/SRC/Indexlist.cpp
#   externals/qpoases/SRC/MessageHandling.cpp
#   externals/qpoases/SRC/QProblem.cpp
#   externals/qpoases/SRC/QProblemB.cpp
#   externals/qpoases/SRC/SubjectTo.cpp
#   externals/qpoases/SRC/Utils.cpp
#   externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
#   model/dynamic_optimize/acado_qpoases_interface.cpp
#   model/dynamic_optimize/acado_integrator.c
#   model/dynamic_optimize/acado_solver.c
#   model/dynamic_optimize/acado_auxiliary_functions.c)

# target_include_directories(mpc_solver PUBLIC
#   model/dynamic_optimize/
#   externals/qpoases
#   externals/qpoases/INCLUDE
#   externals/qpoases/SRC)

# INCLUDE_DIRECTORIES(
#   ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_optimize
#   ${CMAKE_CURRENT_SOURCE_DIR}/../externals/qpoases/INCLUDE)

ADD_EXECUTABLE(   nmpc_model_codegen nmpc_model_generate.cpp )
TARGET_LINK_LIBRARIES( nmpc_model_codegen ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES( nmpc_model_codegen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

# # Minimum required version of cmake
# CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# # Project name and programming languages used
# PROJECT(test_acado)

# # CMake module(s) path
# SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

# # Prerequisites
# FIND_PACKAGE( ACADO REQUIRED )

# # Include directories
# INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

# # Build an executable
# ADD_EXECUTABLE( test_acado nmpc_model_generate.cpp )
# # Linux and OS X
# TARGET_LINK_LIBRARIES( test_acado ${ACADO_SHARED_LIBRARIES} )