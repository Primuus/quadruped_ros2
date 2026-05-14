cmake_minimum_required(VERSION 3.12)
project(logger VERSION 1.0.0 LANGUAGES CXX C)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 平台相关配置
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# 源文件
file(GLOB_RECURSE LOGGER_SOURCES "src/*.cpp")

# 创建静态库
add_library(${PROJECT_NAME} STATIC
    ${LOGGER_SOURCES}
)

# 包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    include
)

# 包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    include
)

# 安装规则
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

# 导出目标
ament_export_targets(${PROJECT_NAME}-targets)
ament_package()