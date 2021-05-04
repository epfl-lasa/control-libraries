cmake_minimum_required(VERSION 3.9)
project(ros2_examples)

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

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

find_library(state_representation REQUIRED)
find_library(dynamical_systems REQUIRED)
find_library(controllers REQUIRED)
find_library(robot_model REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    ${Eigen3_INCLUDE_DIRS}
    ${STATE_REPRESENTATION_INCLUDE_DIR}
    ${DYNAMICAL_SYSTEMS_INCLUDE_DIR}
    ${ROBOT_MODEL_INCLUDE_DIR}
    ${CONTROLLERS_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIR}
    ${OsqpEigen_INCLUDE_DIR}
    /opt/openrobots/include
)

set(DEMOS_SCRIPTS
    task_space_control_loop
)

set(FIXTURE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/fixtures/)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/fixtures/ DESTINATION ${FIXTURE_INSTALL_PATH})

foreach(SCRIPT ${DEMOS_SCRIPTS})
    ament_auto_add_executable(${SCRIPT} scripts/${SCRIPT}.cpp)
    target_link_libraries(${SCRIPT}
        ${PINOCCHIO_LIBRARIES}
        OsqpEigen::OsqpEigen
        osqp::osqp
        state_representation
        dynamical_systems
        robot_model
        controllers
    )
    target_compile_definitions(${SCRIPT} PRIVATE SCRIPT_FIXTURES="${FIXTURE_INSTALL_PATH}")
endforeach()

ament_auto_package()

install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_PREFIX}/launch)

