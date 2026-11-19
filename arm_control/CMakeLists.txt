cmake_minimum_required(VERSION 3.16)

project(arm_control VERSION 0.1.0 LANGUAGES CXX)

# ---------- 全局设置 ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

# ---------- 1. 静态库 ----------
add_library(arm_control STATIC
    src/pid.cpp
    src/lowpass.cpp
)

target_include_directories(arm_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Eigen3 REQUIRED)
target_link_libraries(arm_control PUBLIC Eigen3::Eigen)

# ---------- 2. 可执行程序 ----------
add_executable(demo apps/demo.cpp)
target_link_libraries(demo PRIVATE arm_control)

set_target_properties(demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

# ---------- 3. 单元测试 ----------
enable_testing()

add_executable(test_pid tests/test_pid.cpp)
target_link_libraries(test_pid PRIVATE arm_control)
set_target_properties(test_pid PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)
add_test(NAME test_pid COMMAND test_pid)

add_executable(test_lowpass tests/test_lowpass.cpp)
target_link_libraries(test_lowpass PRIVATE arm_control)
set_target_properties(test_lowpass PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)
add_test(NAME test_lowpass COMMAND test_lowpass)
