cmake_minimum_required(VERSION 3.9)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.

project(state_representation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

set(CORE_SOURCES
  src/MathTools.cpp
  src/State.cpp
  src/Space/SpatialState.cpp
  src/Space/Cartesian/CartesianState.cpp
  src/Space/Cartesian/CartesianPose.cpp
  src/Space/Cartesian/CartesianTwist.cpp
  src/Space/Cartesian/CartesianWrench.cpp
  src/Robot/JointState.cpp
  src/Robot/JointPositions.cpp
  src/Robot/JointVelocities.cpp
  src/Robot/JointTorques.cpp
  src/Robot/Jacobian.cpp
  src/Space/DualQuaternion/DualQuaternionState.cpp
  src/Space/DualQuaternion/DualQuaternionPose.cpp
  src/Space/DualQuaternion/DualQuaternionTwist.cpp
  src/Parameters/ParameterInterface.cpp
  src/Parameters/Parameter.cpp
  src/Parameters/Predicate.cpp
  src/Parameters/Event.cpp
  src/Geometry/Shape.cpp
  src/Geometry/Ellipsoid.cpp
)

add_library(${PROJECT_NAME} SHARED 
  ${CORE_SOURCES}
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

    add_executable(runTestUnits tests/testUnits.cpp)
    target_link_libraries(runTestUnits
      gtest
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestUnits COMMAND runTestUnits)

    add_executable(runTestCartesianState tests/testCartesianState.cpp)
    target_link_libraries(runTestCartesianState
      gtest
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestCartesianState COMMAND runTestCartesianState)

    add_executable(runTestJointState tests/testJointState.cpp)
    target_link_libraries(runTestJointState
      gtest
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestJointState COMMAND runTestJointState)

    add_executable(runTestDualQuaternionState tests/testDualQuaternionState.cpp)
    target_link_libraries(runTestDualQuaternionState
      gtest
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestDualQuaternionState COMMAND runTestDualQuaternionState)

    add_executable(runTestJacobian tests/testJacobian.cpp)
    target_link_libraries(runTestJacobian
      gtest
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestJacobian COMMAND runTestJacobian)

    add_executable(runTestTrajectory tests/testTrajectory.cpp)
    target_link_libraries(runTestTrajectory
      gtest
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestTrajectory COMMAND runTestTrajectory)

    add_executable(runTestParameter tests/testParameter.cpp)
    target_link_libraries(runTestParameter
      gtest
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestParameter COMMAND runTestParameter)

    add_executable(runTestEllipsoid tests/testEllipsoid.cpp)
    target_link_libraries(runTestEllipsoid
      gtest
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestEllipsoid COMMAND runTestEllipsoid)


endif()
