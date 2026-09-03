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
  std_msgs
  tf2
)
find_package(Eigen3 REQUIRED NO_MODULE)
get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_agv_geometry
  CATKIN_DEPENDS agv_msgs geometry_msgs nav_msgs roscpp std_msgs tf2
  DEPENDS Eigen3
)

add_library(multi_agv_geometry
  src/capability_mapper.cpp
  src/dynamic_boundary.cpp
  src/experiment_supervisor.cpp
  src/formal_execution_gate.cpp
  src/lower_channel_controller.cpp
  src/m2b_controller.cpp
  src/path_projector.cpp
  src/planar_support_tracker.cpp
  src/s_curve_path.cpp
  src/state_estimator.cpp
  src/support_geometry.cpp
  src/upper_reference_generator.cpp
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

add_executable(camera_pose_adapter_node src/camera_pose_adapter_node.cpp)
target_include_directories(camera_pose_adapter_node PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(camera_pose_adapter_node
  PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(camera_pose_adapter_node ${catkin_LIBRARIES})
add_dependencies(camera_pose_adapter_node ${catkin_EXPORTED_TARGETS})

add_executable(camera_odom_fusion_node src/camera_odom_fusion_node.cpp)
target_include_directories(camera_odom_fusion_node PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(camera_odom_fusion_node
  PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(camera_odom_fusion_node ${catkin_LIBRARIES})
add_dependencies(camera_odom_fusion_node ${catkin_EXPORTED_TARGETS})

add_executable(multi_agv_controller_node src/multi_agv_controller_node.cpp)
target_include_directories(multi_agv_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(multi_agv_controller_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(multi_agv_controller_node multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(multi_agv_controller_node ${catkin_EXPORTED_TARGETS})

add_executable(formal_fake_algorithm_node src/formal_fake_algorithm_node.cpp)
target_include_directories(formal_fake_algorithm_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(formal_fake_algorithm_node
  PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(formal_fake_algorithm_node
  multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(formal_fake_algorithm_node ${catkin_EXPORTED_TARGETS})

add_executable(formal_algorithm_node src/formal_fake_algorithm_node.cpp)
target_include_directories(formal_algorithm_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(formal_algorithm_node
  PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(formal_algorithm_node
  multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(formal_algorithm_node ${catkin_EXPORTED_TARGETS})

add_executable(single_car_s_pretest_node src/single_car_s_pretest_node.cpp)
target_include_directories(single_car_s_pretest_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(single_car_s_pretest_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(single_car_s_pretest_node multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(single_car_s_pretest_node ${catkin_EXPORTED_TARGETS})

add_executable(three_car_unloaded_bounded_pretest_node
  src/three_car_unloaded_bounded_pretest_node.cpp)
target_include_directories(three_car_unloaded_bounded_pretest_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(three_car_unloaded_bounded_pretest_node
  PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(three_car_unloaded_bounded_pretest_node
  multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(three_car_unloaded_bounded_pretest_node
  ${catkin_EXPORTED_TARGETS})

add_executable(experiment_supervisor_node src/experiment_supervisor_node.cpp)
target_include_directories(experiment_supervisor_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(experiment_supervisor_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(experiment_supervisor_node multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(experiment_supervisor_node ${catkin_EXPORTED_TARGETS})

add_executable(robot2_raised_derating_pretest_node
  src/robot2_raised_derating_pretest_node.cpp)
target_include_directories(robot2_raised_derating_pretest_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(robot2_raised_derating_pretest_node
  PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(robot2_raised_derating_pretest_node
  multi_agv_geometry ${catkin_LIBRARIES})
add_dependencies(robot2_raised_derating_pretest_node ${catkin_EXPORTED_TARGETS})

install(TARGETS multi_agv_geometry path_state_estimator_node camera_pose_adapter_node camera_odom_fusion_node multi_agv_controller_node formal_fake_algorithm_node formal_algorithm_node single_car_s_pretest_node three_car_unloaded_bounded_pretest_node experiment_supervisor_node robot2_raised_derating_pretest_node
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
    test/test_camera_pose_adapter.py
    test/test_central_readonly_e2e.py
    test/test_constant_reference_e2e.py
    test/test_derating_chain_e2e.py
    test/test_formal_fake_algorithm_e2e.py
    test/test_robot2_raised_derating_pretest.py
    test/test_single_car_s_pretest.py
    test/test_three_car_bounded_fault_stop.py
    test/test_three_car_unloaded_bounded_pretest.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/odom_state_estimator.test)
  add_rostest(test/camera_pose_adapter.test)
  add_rostest(test/central_readonly_e2e.test)
  add_rostest(test/constant_reference_e2e.test)
  add_rostest(test/derating_chain_e2e.test)
  add_rostest(test/formal_fake_algorithm_e2e.test)
  add_rostest(test/formal_fake_algorithm_recorder_gate_e2e.test)
  add_rostest(test/formal_fake_algorithm_m2a_r1_e2e.test)
  add_rostest(test/formal_fake_algorithm_m2b_e2e.test)
  add_rostest(test/robot2_raised_derating_pretest.test)
  add_rostest(test/single_car_s_pretest.test)
  add_rostest(test/three_car_bounded_fault_stop.test)
  add_rostest(test/three_car_unloaded_bounded_pretest.test)
  catkin_add_gtest(test_s_curve_path test/test_s_curve_path.cpp)
  catkin_add_gtest(test_support_geometry test/test_support_geometry.cpp)
  catkin_add_gtest(test_path_projector test/test_path_projector.cpp)
  catkin_add_gtest(test_state_estimator test/test_state_estimator.cpp)
  catkin_add_gtest(test_capability_mapper test/test_capability_mapper.cpp)
  catkin_add_gtest(test_planar_support_tracker test/test_planar_support_tracker.cpp)
  catkin_add_gtest(test_experiment_supervisor test/test_experiment_supervisor.cpp)
  catkin_add_gtest(test_formal_execution_gate test/test_formal_execution_gate.cpp)
  catkin_add_gtest(test_dynamic_boundary test/test_dynamic_boundary.cpp)
  catkin_add_gtest(test_upper_reference_generator test/test_upper_reference_generator.cpp)
  catkin_add_gtest(test_lower_channel_controller test/test_lower_channel_controller.cpp)
  catkin_add_gtest(test_m2b_controller test/test_m2b_controller.cpp)
  foreach(test_target test_s_curve_path test_support_geometry test_path_projector test_state_estimator test_capability_mapper test_planar_support_tracker test_experiment_supervisor test_formal_execution_gate test_dynamic_boundary test_upper_reference_generator test_lower_channel_controller test_m2b_controller)
    if(TARGET ${test_target})
      target_compile_options(${test_target} PRIVATE -Wall -Wextra -Wpedantic)
      target_compile_definitions(${test_target} PRIVATE
        MULTI_AGV_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
      target_link_libraries(${test_target} multi_agv_geometry gtest_main)
    endif()
  endforeach()
endif()
