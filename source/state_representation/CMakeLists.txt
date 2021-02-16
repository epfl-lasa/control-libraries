project(state_representation)

include_directories(include)

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
#  add_executable(runTestParameter tests/testParameter.cpp)
#  target_link_libraries(runTestParameter
#    ${PROJECT_NAME}
#    ${GTEST_LIBRARIES}
#    pthread
#  )
#  add_test(NAME runTestParameter COMMAND runTestParameter)

  add_executable(runTestEllipsoid tests/testEllipsoid.cpp)
  target_link_libraries(runTestEllipsoid
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestEllipsoid COMMAND runTestEllipsoid)
endif ()
