cmake_minimum_required(VERSION 3.8)
project(kbd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadruped REQUIRED)

# Create keyboard control executable
add_executable(kbd src/kbd.cpp)

# Link libraries
ament_target_dependencies(kbd
  rclcpp
  std_msgs
  quadruped
)

install(TARGETS
  kbd
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()