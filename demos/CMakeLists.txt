cmake_minimum_required(VERSION 3.9)
project(control_libraries_demos)

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

foreach(SCRIPT ${DEMOS_SCRIPTS})
    add_executable(${SCRIPT} scripts/${SCRIPT}.cpp)
    target_link_libraries(${SCRIPT}
        ${PINOCCHIO_LIBRARIES}
        OsqpEigen::OsqpEigen
        osqp::osqp
        state_representation
        dynamical_systems
        robot_model
        controllers
    )
    install(TARGETS ${SCRIPT}
      RUNTIME DESTINATION bin
    )
endforeach()