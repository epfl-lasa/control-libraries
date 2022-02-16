project(controllers)

find_library(robot_model REQUIRED)

get_target_property(STATE_REPRESENTATION_INCLUDE_DIR state_representation INCLUDE_DIRECTORIES)
get_target_property(ROBOT_MODEL_INCLUDE_DIR robot_model INCLUDE_DIRECTORIES)
include_directories(
  include
  ${STATE_REPRESENTATION_INCLUDE_DIR}
  ${ROBOT_MODEL_INCLUDE_DIR}
)

set(CORE_SOURCES
  src/ControllerFactory.cpp
  src/IController.cpp
  src/impedance/CompliantTwist.cpp
  src/impedance/NewDissipative.cpp
  src/impedance/NewImpedance.cpp
  src/impedance/NewVelocityImpedance.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${CORE_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  state_representation
  robot_model
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
  add_executable(test_controllers test/test_controllers.cpp)
  file(GLOB_RECURSE MODULE_TEST_SOURCES test/tests test_*.cpp)
  target_sources(test_controllers PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(test_controllers
    ${PROJECT_NAME}
    state_representation
    ${GTEST_LIBRARIES}
    pthread
  )
  target_compile_definitions(test_controllers PRIVATE TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/")
  add_test(NAME test_controllers COMMAND test_controllers)
endif ()
