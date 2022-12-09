set(LIBRARY_NAME state_representation)

set(EIGEN3_VERSION 3.4.0)
find_package(Eigen3 ${EIGEN3_VERSION} REQUIRED)
set(EIGEN_MPL2_ONLY 1)

set(CORE_SOURCES
  src/MathTools.cpp
  src/State.cpp
  src/space/SpatialState.cpp
  src/space/cartesian/CartesianState.cpp
  src/space/cartesian/CartesianPose.cpp
  src/space/cartesian/CartesianTwist.cpp
  src/space/cartesian/CartesianAcceleration.cpp
  src/space/cartesian/CartesianWrench.cpp
  src/space/joint/JointState.cpp
  src/space/joint/JointPositions.cpp
  src/space/joint/JointVelocities.cpp
  src/space/joint/JointAccelerations.cpp
  src/space/joint/JointTorques.cpp
  src/space/Jacobian.cpp
  src/parameters/Event.cpp
  src/parameters/Parameter.cpp
  src/parameters/ParameterInterface.cpp
  src/parameters/ParameterMap.cpp
  src/parameters/Predicate.cpp
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


add_library(${LIBRARY_NAME} SHARED ${CORE_SOURCES})
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# add include directories
target_include_directories(${LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# install the target and create export-set
install(TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# generate and install export file
install(EXPORT ${LIBRARY_NAME}_targets
  FILE ${PROJECT_NAME}_${LIBRARY_NAME}_targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if (BUILD_TESTING)
  add_executable(test_${LIBRARY_NAME} test/test_state_representation.cpp)
  file(GLOB_RECURSE MODULE_TEST_SOURCES test/tests test_*.cpp)
  target_sources(test_${LIBRARY_NAME} PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(test_${LIBRARY_NAME}
    ${LIBRARY_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME test_${LIBRARY_NAME} COMMAND test_${LIBRARY_NAME})
endif ()

if(${PKG_CONFIG_FOUND})
  set(PKG_NAME ${LIBRARY_NAME})
  set(PKG_DESC "This library provides a set of classes to represent states in Cartesian and joint space.")
  set(PKG_EXTERNAL_DEPS "eigen3 >= ${EIGEN3_VERSION}")
  set(pkg_conf_file "state_representation.pc")
  configure_file("${CMAKE_SOURCE_DIR}/library_template.pc.in" "${CMAKE_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES "${CMAKE_BINARY_DIR}/${pkg_conf_file}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
endif()
