cmake_minimum_required(VERSION 3.9)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.

project(robot_model)

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
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIR}
  /home/ros2/openrobots/include
  ${OsqpEigen_INCLUDE_DIR}
)

set(CORE_SOURCES
  src/Model.cpp
)

add_library(${PROJECT_NAME} SHARED 
  ${CORE_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PINOCCHIO_LIBRARIES}
  OsqpEigen::OsqpEigen
  osqp::osqp
  state_representation
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if (runtests)
  if (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    add_definitions(-D__GLIBCXX__)
  endif (APPLE)

  add_subdirectory(lib/googletest)

  enable_testing()

  # Include the gtest library. gtest_SOURCE_DIR is available due to
  # 'project(gtest)' above.
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  add_executable(runTestRobotModel tests/testRobotModel.cpp)
  target_link_libraries(runTestRobotModel
    gtest 
    gtest_main
    ${PROJECT_NAME}
  )
  add_test(NAME runTestRobotModel COMMAND runTestRobotModel)
endif()
