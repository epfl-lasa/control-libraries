project(dynamical_systems)

get_target_property(STATE_REPRESENTATION_INCLUDE_DIR state_representation INCLUDE_DIRECTORIES)
include_directories(
  include
  ${STATE_REPRESENTATION_INCLUDE_DIR}
)

set(CORE_SOURCES
  src/DynamicalSystem.cpp
  src/Circular.cpp
  src/Linear.cpp
  src/Ring.cpp
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
  add_executable(test_dynamical_systems test/test_dynamical_systems.cpp)
  file(GLOB_RECURSE MODULE_TEST_SOURCES test/tests test_*.cpp)
  target_sources(test_dynamical_systems PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(test_dynamical_systems
    ${PROJECT_NAME}
    state_representation
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME test_dynamical_systems COMMAND test_dynamical_systems)
endif ()
