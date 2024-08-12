cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(ModelTests)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加Eigen库路径
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 包含头文件目录
include_directories(include)

# 添加源文件
add_executable(ModelTests src/test_whole_body_controller.cpp src/model.cpp)

# 链接Eigen库
target_link_libraries(ModelTests Eigen3::Eigen)