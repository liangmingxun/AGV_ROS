cmake_minimum_required(VERSION 3.0.2)
project(chassis_controller)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_BUILD_TYPE Release)

## 查找catkin和依赖包
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    common
    message_generation
    tf
)

## 查找Boost依赖（用于asio）
find_package(Boost REQUIRED COMPONENTS
    system
    thread)

find_package(Eigen3 REQUIRED NO_MODULE)

add_message_files(
  FILES
  ImuData.msg
  ControlCommand.msg
  RobotState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES chassis_controller
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        common
        message_runtime
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



# 搜索各子目录下的 .cpp 文件
file(GLOB OTHER_SRC "src/*.cpp")
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
set(ALL_SRC
    ${OTHER_SRC}
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

# 添加可执行文件
add_executable(chassis_controller ${ALL_SRC})

message(STATUS "ALL_SRC contains:")
foreach(file ${ALL_SRC})
    message(STATUS " - ${file}")
endforeach()

target_link_libraries(chassis_controller
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)