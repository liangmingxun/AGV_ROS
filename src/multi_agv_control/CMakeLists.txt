cmake_minimum_required(VERSION 3.0.2)
project(multi_agv_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_agv_geometry
  DEPENDS Eigen3
)

add_library(multi_agv_geometry
  src/s_curve_path.cpp
  src/support_geometry.cpp
)
target_include_directories(multi_agv_geometry
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)
target_compile_options(multi_agv_geometry PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(multi_agv_geometry Eigen3::Eigen)

install(TARGETS multi_agv_geometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_s_curve_path test/test_s_curve_path.cpp)
  catkin_add_gtest(test_support_geometry test/test_support_geometry.cpp)
  foreach(test_target test_s_curve_path test_support_geometry)
    if(TARGET ${test_target})
      target_compile_options(${test_target} PRIVATE -Wall -Wextra -Wpedantic)
      target_link_libraries(${test_target} multi_agv_geometry gtest_main)
    endif()
  endforeach()
endif()
