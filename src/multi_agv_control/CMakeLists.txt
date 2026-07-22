cmake_minimum_required(VERSION 3.0.2)
project(multi_agv_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(catkin REQUIRED COMPONENTS
  agv_msgs
  geometry_msgs
  nav_msgs
  roscpp
)
find_package(Eigen3 REQUIRED NO_MODULE)
get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_agv_geometry
  CATKIN_DEPENDS agv_msgs geometry_msgs nav_msgs roscpp
  DEPENDS Eigen3
)

add_library(multi_agv_geometry
  src/capability_mapper.cpp
  src/path_projector.cpp
  src/planar_support_tracker.cpp
  src/s_curve_path.cpp
  src/state_estimator.cpp
  src/support_geometry.cpp
)
target_include_directories(multi_agv_geometry
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_compile_options(multi_agv_geometry PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(multi_agv_geometry Eigen3::Eigen ${catkin_LIBRARIES})

add_executable(path_state_estimator_node src/path_state_estimator_node.cpp)
target_include_directories(path_state_estimator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(path_state_estimator_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(path_state_estimator_node multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(path_state_estimator_node ${catkin_EXPORTED_TARGETS})

add_executable(multi_agv_controller_node src/multi_agv_controller_node.cpp)
target_include_directories(multi_agv_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(multi_agv_controller_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(multi_agv_controller_node multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(multi_agv_controller_node ${catkin_EXPORTED_TARGETS})

install(TARGETS multi_agv_geometry path_state_estimator_node multi_agv_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS
    test/test_odom_state_estimator.py
    test/test_constant_reference_e2e.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/odom_state_estimator.test)
  add_rostest(test/constant_reference_e2e.test)
  catkin_add_gtest(test_s_curve_path test/test_s_curve_path.cpp)
  catkin_add_gtest(test_support_geometry test/test_support_geometry.cpp)
  catkin_add_gtest(test_path_projector test/test_path_projector.cpp)
  catkin_add_gtest(test_state_estimator test/test_state_estimator.cpp)
  catkin_add_gtest(test_capability_mapper test/test_capability_mapper.cpp)
  catkin_add_gtest(test_planar_support_tracker test/test_planar_support_tracker.cpp)
  foreach(test_target test_s_curve_path test_support_geometry test_path_projector test_state_estimator test_capability_mapper test_planar_support_tracker)
    if(TARGET ${test_target})
      target_compile_options(${test_target} PRIVATE -Wall -Wextra -Wpedantic)
      target_link_libraries(${test_target} multi_agv_geometry gtest_main)
    endif()
  endforeach()
endif()
