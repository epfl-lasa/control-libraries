cmake_minimum_required(VERSION 3.9)
project(control_loop_examples)

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
include_directories(/opt/openrobots/include)

find_package(control_libraries 6.0.3 CONFIG REQUIRED)

set(DEMOS_SCRIPTS
  task_space_control_loop
  robot_kinematics_control_loop
)

set(FIXTURE_INSTALL_PATH /etc/fixtures/)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/ DESTINATION ${FIXTURE_INSTALL_PATH})

foreach(SCRIPT ${DEMOS_SCRIPTS})
  add_executable(${SCRIPT} cpp_scripts/${SCRIPT}.cpp)
  target_link_libraries(${SCRIPT} ${control_libraries_LIBRARIES})
  target_compile_definitions(${SCRIPT} PRIVATE SCRIPT_FIXTURES="${FIXTURE_INSTALL_PATH}")
endforeach()