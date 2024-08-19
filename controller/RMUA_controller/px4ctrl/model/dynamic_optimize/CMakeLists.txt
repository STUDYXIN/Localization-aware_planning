cmake_minimum_required(VERSION 2.8)
project(MyProject)

ADD_EXECUTABLE(test_nmpc test.cpp)

target_sources(test_nmpc
PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/acado_common.h
${CMAKE_CURRENT_SOURCE_DIR}/acado_auxiliary_functions.c
${CMAKE_CURRENT_SOURCE_DIR}/acado_solver.c
${CMAKE_CURRENT_SOURCE_DIR}/acado_integrator.c
${CMAKE_CURRENT_SOURCE_DIR}/acado_qpoases_interface.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/Bounds.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/Constraints.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/CyclingManager.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/Indexlist.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/MessageHandling.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/QProblem.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/QProblemB.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/SubjectTo.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/Utils.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

target_include_directories(test_nmpc
PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases
  ${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../externals/qpoases/SRC

)