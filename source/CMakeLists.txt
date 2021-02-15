cmake_minimum_required(VERSION 3.9)

project(control_libraries)

option(runtests "Build all tests." OFF)
option(BUILD_DYNAMICAL_SYSTEMS "Build and install dynamical systems library" ON)
option(BUILD_ROBOT_MODEL "Build and install robot model library" OFF)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_subdirectory(state_representation)

if(BUILD_DYNAMICAL_SYSTEMS)
    add_subdirectory(dynamical_systems)
endif()

if(BUILD_ROBOT_MODEL)
    add_subdirectory(robot_model)
endif()

if(runtests)
    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
endif()