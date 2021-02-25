project(controllers)

get_target_property(STATE_REPRESENTATION_INCLUDE_DIR state_representation INCLUDE_DIRECTORIES)
include_directories(
  include
  ${STATE_REPRESENTATION_INCLUDE_DIR}
)

set(CORE_SOURCES
  src/impedance/Impedance.cpp
  src/impedance/VelocityImpedance.cpp
  src/impedance/Dissipative.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${CORE_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
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
  add_executable(test_controllers test/test_controllers.cpp)
  file(GLOB_RECURSE MODULE_TEST_SOURCES test/tests test_*.cpp)
  target_sources(test_controllers PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(test_controllers
    ${PROJECT_NAME}
    state_representation
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME test_controllers COMMAND test_controllers)
endif ()
