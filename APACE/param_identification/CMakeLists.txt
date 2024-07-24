cmake_minimum_required(VERSION 3.0.2)
project(param_identification)


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  eigen_conversions
  # visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs quadrotor_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pub_trigger src/pub_trigger.cpp)
add_executable(pub_unique_traj src/pub_unique_traj.cpp)

target_link_libraries(pub_trigger
  ${EIGEN3_LIBRARIES} 
  ${catkin_LIBRARIES}
)

target_link_libraries(pub_unique_traj
  ${EIGEN3_LIBRARIES} 
  ${catkin_LIBRARIES}
)
