project(state_representation)

include_directories(include)

set(CORE_SOURCES
  src/MathTools.cpp
  src/State.cpp
  src/space/SpatialState.cpp
  src/space/cartesian/CartesianState.cpp
  src/space/cartesian/CartesianPose.cpp
  src/space/cartesian/CartesianTwist.cpp
  src/space/cartesian/CartesianAcceleration.cpp
  src/space/cartesian/CartesianWrench.cpp
  src/robot/JointState.cpp
  src/robot/JointPositions.cpp
  src/robot/JointVelocities.cpp
  src/robot/JointAccelerations.cpp
  src/robot/JointTorques.cpp
  src/robot/Jacobian.cpp
  src/parameters/ParameterInterface.cpp
  src/parameters/Parameter.cpp
  src/parameters/Predicate.cpp
  src/parameters/Event.cpp
  src/geometry/Shape.cpp
  src/geometry/Ellipsoid.cpp
)

if (EXPERIMENTAL_FEATURES)
  list(APPEND CORE_SOURCES
    src/space/dual_quaternion/DualQuaternionState.cpp
    src/space/dual_quaternion/DualQuaternionPose.cpp
    src/space/dual_quaternion/DualQuaternionTwist.cpp
  )
endif ()

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
  add_executable(test_state_representation test/test_state_representation.cpp)
  file(GLOB_RECURSE MODULE_TEST_SOURCES test/tests test_*.cpp)
  target_sources(test_state_representation PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(test_state_representation
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME test_state_representation COMMAND test_state_representation)
endif ()
