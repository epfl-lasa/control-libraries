cmake_minimum_required(VERSION 3.9)
project(ros_examples)

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

find_package(control_libraries 5.0.0 CONFIG REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
add_compile_definitions(BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_compile_definitions(BOOST_MPL_LIMIT_LIST_SIZE=30)

catkin_package(
    CATKIN_DEPENDS roscpp tf
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    /opt/openrobots/include
)

set(DEMOS_SCRIPTS
    task_space_control_loop
    joint_space_velocity_control
)
set(FIXTURE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/fixtures/)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/fixtures/ DESTINATION ${FIXTURE_INSTALL_PATH})

foreach(SCRIPT ${DEMOS_SCRIPTS})
    add_executable(${SCRIPT} scripts/${SCRIPT}.cpp)
    target_link_libraries(${SCRIPT}
        ${control_libraries_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    target_compile_definitions(${SCRIPT} PRIVATE SCRIPT_FIXTURES="${FIXTURE_INSTALL_PATH}")
    install(TARGETS ${SCRIPT}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)