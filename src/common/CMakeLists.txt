cmake_minimum_required(VERSION 3.0.2)
project(common)

## 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES common
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${NLopt_INCLUDE_DIRS}
)

## 编译库
add_library(common
  src/common.cpp
)

target_link_libraries(common
  ${catkin_LIBRARIES}
)