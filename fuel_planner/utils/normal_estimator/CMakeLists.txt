cmake_minimum_required(VERSION 2.8.3)
project(normal_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Python3 and NumPy setup
find_package(Python3 COMPONENTS Interpreter NumPy REQUIRED)
message(STATUS "Python3 include dir: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 NumPy include dir: ${Python3_NumPy_INCLUDE_DIRS}")

# Set matplotlib-cpp path (replace with actual path)
set(MATPLOTLIBCPP_DIR "/path/to/matplotlib-cpp")
include_directories(${MATPLOTLIBCPP_DIR})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES normal_estimator
)

include_directories(
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}  # Include Python3 directories
    ${Python3_NumPy_INCLUDE_DIRS}  # Include NumPy directories
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(normal_estimator
    src/normal_estimator.cpp
)

# Link with Python3 libraries
target_link_libraries(normal_estimator
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    Python3::Python  # Link against Python3 interpreter
    Python3::NumPy   # Link against NumPy
)
