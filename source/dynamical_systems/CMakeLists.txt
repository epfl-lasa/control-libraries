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
  add_executable(runTestLinear tests/testLinear.cpp)
  target_link_libraries(runTestLinear
    ${PROJECT_NAME}
    state_representation
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestLinear COMMAND runTestLinear)

  add_executable(runTestCircular tests/testCircular.cpp)
  target_link_libraries(runTestCircular
    ${PROJECT_NAME}
    state_representation
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME runTestCircular COMMAND runTestCircular)
endif ()
