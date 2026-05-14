cmake_minimum_required(VERSION 3.12)
project(quadruped
    VERSION 1.0.0
    LANGUAGES CXX C
    DESCRIPTION "Unified quadruped robot control system"
)

# ================== Compiler Options and Standards ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Platform related configurations
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ================== Project Directory Structure ==================
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ================== Find Dependencies ==================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(logger REQUIRED)

# ================== Source Files Configuration ==================
# Common library sources (header-only if no sources)
file(GLOB_RECURSE COMMON_SOURCES 
    "src/common/*.cpp"
)

# Math library sources
file(GLOB_RECURSE MATH_SOURCES 
    "src/math/*.cpp"
)

# Motor library sources
file(GLOB_RECURSE MOTOR_SOURCES 
    "src/motor/*.cpp"
)

# Leg library sources
file(GLOB_RECURSE LEG_SOURCES 
    "src/leg/*.cpp"
)

# FSM library sources
file(GLOB_RECURSE FSM_SOURCES 
    "src/fsm/*.cpp"
    "src/config/*.cpp"
)

# Motion library sources
file(GLOB_RECURSE MOTION_SOURCES 
    "src/motion/*.cpp"
)

# Phase library sources
file(GLOB_RECURSE PHASE_SOURCES 
    "src/phase/*.cpp"
)

# Control interface sources
file(GLOB_RECURSE INTERFACE_SOURCES 
    "src/control/ros2_interface.cpp"
    "src/control/main.cpp"
    "src/control/hand.cpp"
)

# Control node sources
file(GLOB_RECURSE NODE_SOURCES 
    "src/control/quadruped.cpp"
)

# ================== Create Libraries ==================

# Common library (header-only if no sources)
if(COMMON_SOURCES)
    add_library(quadruped_common STATIC
        ${COMMON_SOURCES}
    )
    target_include_directories(quadruped_common PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        include
    )
    target_include_directories(quadruped_common PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
else()
    add_library(quadruped_common INTERFACE)
    target_include_directories(quadruped_common INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIR}
    )
endif()

# Math library
add_library(quadruped_math SHARED
    ${MATH_SOURCES}
)
target_include_directories(quadruped_math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(quadruped_math
    rclcpp
    std_msgs
    geometry_msgs
)
target_link_libraries(quadruped_math
    quadruped_common
    logger::logger
)

# Motor library
add_library(quadruped_motor SHARED
    ${MOTOR_SOURCES}
)
target_include_directories(quadruped_motor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(quadruped_motor
    rclcpp
    std_msgs
    geometry_msgs
)
target_link_libraries(quadruped_motor
    quadruped_common
    quadruped_math
    logger::logger
)

# Leg library
add_library(quadruped_leg SHARED
    ${LEG_SOURCES}
)
target_include_directories(quadruped_leg PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(quadruped_leg
    rclcpp
    std_msgs
    geometry_msgs
)
target_link_libraries(quadruped_leg
    quadruped_common
    quadruped_math
    quadruped_motor
    logger::logger
)

# FSM library
add_library(quadruped_fsm SHARED
    ${FSM_SOURCES}
)
target_include_directories(quadruped_fsm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(quadruped_fsm
    rclcpp
    std_msgs
    geometry_msgs
)
target_link_libraries(quadruped_fsm
    quadruped_common
    quadruped_math
    quadruped_leg
    quadruped_motion
    logger::logger
)

# Motion library
add_library(quadruped_motion SHARED
    ${MOTION_SOURCES}
)
target_include_directories(quadruped_motion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(quadruped_motion
    rclcpp
    std_msgs
    geometry_msgs
)
target_link_libraries(quadruped_motion
    quadruped_common
    quadruped_math
    quadruped_leg
    quadruped_motor
    logger::logger
)

# Phase library
add_library(quadruped_phase SHARED
    ${PHASE_SOURCES}
)
target_include_directories(quadruped_phase PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(quadruped_phase
    rclcpp
    std_msgs
    geometry_msgs
)
target_link_libraries(quadruped_phase
    quadruped_common
    quadruped_math
    quadruped_leg
    logger::logger
)

# Control interface library
add_library(quadruped_control_interface SHARED
    ${INTERFACE_SOURCES}
)
target_include_directories(quadruped_control_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(quadruped_control_interface
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    diagnostic_msgs
)
target_link_libraries(quadruped_control_interface
    quadruped_common
    quadruped_math
    quadruped_motor
    quadruped_leg
    quadruped_fsm
    quadruped_phase
    quadruped_motion
    logger::logger
    quadruped__rosidl_typesupport_cpp
    quadruped__rosidl_generator_c
)

# Control node executable
add_executable(quadruped_node
    ${NODE_SOURCES}
)
target_include_directories(quadruped_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(quadruped_node
    rclcpp
    std_msgs
    geometry_msgs
    diagnostic_msgs
)
target_link_libraries(quadruped_node
    quadruped_control_interface
    quadruped__rosidl_typesupport_cpp
    quadruped__rosidl_generator_c
)

# ================== Test Executable ==================
# 独立的新功能测试入口，不依赖ROS2节点启动
add_executable(test_new_feature
    test/test_new_feature.cpp
)
target_include_directories(test_new_feature PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(test_new_feature
    rclcpp
    std_msgs
    geometry_msgs
)
target_link_libraries(test_new_feature
    quadruped_common
    quadruped_math
    quadruped_motor
    quadruped_leg
    quadruped_motion
    quadruped_fsm
    quadruped_phase
    logger::logger
)

# ================== ROS2 Messages and Services ==================
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MotorCommand.msg"
    "msg/RobotState.msg"
    "msg/IMUData.msg"
    "msg/GaitCommand.msg"
    "msg/JoystickCommand.msg"
    "srv/ChangeGait.srv"
    DEPENDENCIES std_msgs geometry_msgs diagnostic_msgs builtin_interfaces
)

# Ensure interface libraries depend on message generation
add_dependencies(quadruped_control_interface quadruped__rosidl_typesupport_cpp)
add_dependencies(quadruped_node quadruped__rosidl_typesupport_cpp)

# ================== Installation Rules ==================
install(TARGETS
    quadruped_math
    quadruped_motor
    quadruped_leg
    quadruped_fsm
    quadruped_motion
    quadruped_phase
    quadruped_control_interface
    quadruped_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/real_robot_terminals.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME real_robot_terminals
)

install(DIRECTORY
    include/
    DESTINATION include
)

install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    msg/
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    srv/
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/
)

# ================== Export Dependencies ==================
ament_export_dependencies(
    rclcpp
    std_msgs
    geometry_msgs
    diagnostic_msgs
    builtin_interfaces
    Eigen3
    logger
)

ament_export_include_directories(include)
ament_export_libraries(
    quadruped_math
    quadruped_motor
    quadruped_leg
    quadruped_fsm
    quadruped_motion
    quadruped_phase
    quadruped_control_interface
)

ament_package()
