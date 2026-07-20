cmake_minimum_required(VERSION 3.0.2)
project(chassis_controller)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

## 查找catkin和依赖包
find_package(catkin REQUIRED COMPONENTS
    agv_msgs
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    common
    tf
)

## 查找Boost依赖（用于asio）
find_package(Boost REQUIRED COMPONENTS
    system
    thread)

find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES chassis_controller_core
    CATKIN_DEPENDS
        agv_msgs
        geometry_msgs
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
        std_srvs
        common
        tf
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MathLib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Quaternion
    ${CMAKE_CURRENT_SOURCE_DIR}/src/QEKF
    ${CMAKE_CURRENT_SOURCE_DIR}/src/QEKF/MATLABCoder
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MATLABCoderInit
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Parameters
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FirstOrderLPF  # 这里应该是目录
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)



# 搜索旧底盘算法依赖的各子目录 .cpp 文件。纯核心类显式列出，避免 GLOB
# 把它们重复编译进节点。
file(GLOB QEKF_SRC              "src/QEKF/*.cpp")
file(GLOB QEKF_MATLAB_SRC       "src/QEKF/MATLABCoder/*.cpp")
file(GLOB QUATERNION_SRC        "src/Quaternion/*.cpp")
file(GLOB MATLAB_INIT_SRC       "src/MATLABCoderInit/*.cpp")
file(GLOB MATHLIB_SRC           "src/MathLib/*.cpp")
# file(GLOB MODELMATRICES         "src/ModelMatrices/*.cpp")
# file(GLOB IIR_SRC               "src/IIR/*.cpp")
# file(GLOB LQR_SRC               "src/LQR/*.cpp")
# file(GLOB KINEMATICS_SRC        "src/Kinematics/*.cpp")
# file(GLOB KINEMATICS_MATLAB_SRC "src/Kinematics/MATLABCoder/*.cpp")
# file(GLOB VELOCITYEKF_SRC       "src/VelocityEKF/*.cpp")
# file(GLOB VELOCITYEKF_MATLAB_SRC "src/VelocityEKF/MATLABCoder/*.cpp")
# file(GLOB COMEKF_SRC            "src/COMEKF/*.cpp")
# file(GLOB COMEKF_MATLAB_SRC     "src/COMEKF/MATLABCoder/*.cpp")
# file(GLOB WHEELSLIPDETECTOR_SRC "src/WheelSlipDetector/*.cpp")
file(GLOB FIRSTORDERLPF_SRC     "src/FirstOrderLPF/*.cpp")

# 合并所有源文件
set(LEGACY_SRC
    ${QEKF_SRC}
    ${QEKF_MATLAB_SRC}
    ${QUATERNION_SRC}
    ${MATLAB_INIT_SRC}
    ${MATHLIB_SRC}
    # ${MODELMATRICES}
    # ${IIR_SRC}
    # ${LQR_SRC}
    # ${KINEMATICS_SRC}
    # ${KINEMATICS_MATLAB_SRC}
    # ${VELOCITYEKF_SRC}
    # ${VELOCITYEKF_MATLAB_SRC}
    # ${COMEKF_SRC}
    # ${COMEKF_MATLAB_SRC}
    # ${WHEELSLIPDETECTOR_SRC}
    ${FIRSTORDERLPF_SRC}
)

add_library(chassis_controller_core
    src/wheel_command_limiter.cpp
    src/derating_profile.cpp
    src/odometry_integrator.cpp
    src/chassis_core.cpp
)
target_link_libraries(chassis_controller_core ${catkin_LIBRARIES})

add_executable(chassis_controller src/main.cpp ${LEGACY_SRC})

target_link_libraries(chassis_controller
    chassis_controller_core
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)

add_dependencies(chassis_controller ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_gtest(test_wheel_command_limiter
        test/test_wheel_command_limiter.cpp)
    catkin_add_gtest(test_derating_profile
        test/test_derating_profile.cpp)
    catkin_add_gtest(test_odometry_integrator
        test/test_odometry_integrator.cpp)
    catkin_add_gtest(test_chassis_core
        test/test_chassis_core.cpp)
    foreach(test_target
            test_wheel_command_limiter
            test_derating_profile
            test_odometry_integrator
            test_chassis_core)
        if(TARGET ${test_target})
            target_link_libraries(${test_target}
                chassis_controller_core
                gtest_main)
        endif()
    endforeach()
    add_rostest(test/chassis_node.test)
endif()
