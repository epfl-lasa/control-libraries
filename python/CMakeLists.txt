cmake_minimum_required(VERSION 3.10)

project(pycontrol_libraries)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_COMPILER c++)

execute_process(COMMAND python -m pybind11 --cmakedir OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(pybind11 CONFIG)

find_library(state_representation REQUIRED)
find_library(dynamical_systems REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

pybind11_add_module(py_state_representation source/py_state_representation.cpp)
target_link_libraries(py_state_representation PRIVATE state_representation Eigen3::Eigen)

set(CMAKE_INSTALL_PREFIX /)
install(TARGETS py_state_representation
    LIBRARY DESTINATION lib
)