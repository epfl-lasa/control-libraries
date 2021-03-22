project(robot_model)

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

get_target_property(STATE_REPRESENTATION_INCLUDE_DIR state_representation INCLUDE_DIRECTORIES)
include_directories(
  include
  ${STATE_REPRESENTATION_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIR}
  /opt/openrobots/include
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

if (BUILD_TESTING)
  add_executable(test_robot_model test/test_robot_model.cpp)
  file(GLOB_RECURSE MODULE_TEST_SOURCES test/tests test_*.cpp)
  target_sources(test_robot_model PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(test_robot_model
    ${PROJECT_NAME}
    state_representation
    ${GTEST_LIBRARIES}
    pthread
  )
  target_compile_definitions(test_robot_model PRIVATE TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/tests/fixtures/")
  add_test(NAME test_robot_model COMMAND test_robot_model)
endif ()
