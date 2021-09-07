cmake_minimum_required(VERSION 3.9)

project(control-libraries VERSION 4.0.0)

# Build options
option(BUILD_TESTING "Build all tests." OFF)
option(BUILD_CONTROLLERS "Build and install controllers library" ON)
option(BUILD_DYNAMICAL_SYSTEMS "Build and install dynamical systems library" ON)
option(BUILD_ROBOT_MODEL "Build and install robot model library" ON)
option(EXPERIMENTAL_FEATURES "Include experimental features" OFF)

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

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  if (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    add_definitions(-D__GLIBCXX__)
  endif (APPLE)
else()
  find_package(GTest QUIET)
endif()

add_subdirectory(state_representation)

if(BUILD_CONTROLLERS)
  add_subdirectory(controllers)
endif()

if(BUILD_DYNAMICAL_SYSTEMS)
  add_subdirectory(dynamical_systems)
endif()

if(BUILD_ROBOT_MODEL)
  add_subdirectory(robot_model)
endif()
