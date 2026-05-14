cmake_minimum_required(VERSION 3.5)
project(robot_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf worlds config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
