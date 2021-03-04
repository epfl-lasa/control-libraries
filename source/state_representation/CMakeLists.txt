project(state_representation)

include_directories(include)

set(CORE_SOURCES
  src/MathTools.cpp
  src/State.cpp
        src/space/SpatialState.cpp
        src/space/cartesian/CartesianState.cpp
        src/space/cartesian/CartesianPose.cpp
        src/space/cartesian/CartesianTwist.cpp
        src/space/cartesian/CartesianWrench.cpp
        src/robot/JointState.cpp
        src/robot/JointPositions.cpp
        src/robot/JointVelocities.cpp
        src/robot/JointTorques.cpp
        src/robot/Jacobian.cpp
        src/space/DualQuaternion/DualQuaternionState.cpp
        src/space/DualQuaternion/DualQuaternionPose.cpp
        src/space/DualQuaternion/DualQuaternionTwist.cpp
        src/parameters/ParameterInterface.cpp
        src/parameters/Parameter.cpp
        src/parameters/Predicate.cpp
        src/parameters/Event.cpp
        src/geometry/Shape.cpp
        src/geometry/Ellipsoid.cpp
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

if (BUILD_TESTING)
  add_executable(runTestUnits tests/testUnits.cpp)
  target_link_libraries(runTestUnits
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestUnits COMMAND runTestUnits)

  add_executable(runTestCartesianState tests/testCartesianState.cpp)
  target_link_libraries(runTestCartesianState
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestCartesianState COMMAND runTestCartesianState)

  add_executable(runTestJointState tests/testJointState.cpp)
  target_link_libraries(runTestJointState
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestJointState COMMAND runTestJointState)

  add_executable(runTestDualQuaternionState tests/testDualQuaternionState.cpp)
  target_link_libraries(runTestDualQuaternionState
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestDualQuaternionState COMMAND runTestDualQuaternionState)

  add_executable(runTestJacobian tests/testJacobian.cpp)
  target_link_libraries(runTestJacobian
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestJacobian COMMAND runTestJacobian)

  add_executable(runTestTrajectory tests/testTrajectory.cpp)
  target_link_libraries(runTestTrajectory
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestTrajectory COMMAND runTestTrajectory)

  # testParameter fails to build for accessing private member line 23 |  EXPECT_FALSE(e.previous_predicate_value_);
  add_executable(runTestParameter tests/testParameter.cpp)
  target_link_libraries(runTestParameter
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestParameter COMMAND runTestParameter)

  add_executable(runTestEllipsoid tests/testEllipsoid.cpp)
  target_link_libraries(runTestEllipsoid
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestEllipsoid COMMAND runTestEllipsoid)
endif ()
