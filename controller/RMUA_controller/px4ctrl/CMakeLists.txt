cmake_minimum_required(VERSION 3.0.2)
project(px4ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mavros
  quadrotor_msgs
  sensor_msgs
  uav_utils
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include 
)

## Declare a C++ executable
add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)


## Add cmake target dependencies of the executable
add_dependencies(px4ctrl_node quadrotor_msgs)


target_sources(px4ctrl_node
PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/model/dynamic_optimize/acado_common.h
${CMAKE_CURRENT_SOURCE_DIR}/model/dynamic_optimize/acado_auxiliary_functions.c
${CMAKE_CURRENT_SOURCE_DIR}/model/dynamic_optimize/acado_solver.c
${CMAKE_CURRENT_SOURCE_DIR}/model/dynamic_optimize/acado_integrator.c
${CMAKE_CURRENT_SOURCE_DIR}/model/dynamic_optimize/acado_qpoases_interface.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/Bounds.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/Constraints.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/CyclingManager.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/Indexlist.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/MessageHandling.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/QProblem.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/QProblemB.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/SubjectTo.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/Utils.cpp
${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

target_include_directories(px4ctrl_node
PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/model/dynamic_optimize
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC

)
## Specify libraries to link a library or executable target against
target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py scripts/odom_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
