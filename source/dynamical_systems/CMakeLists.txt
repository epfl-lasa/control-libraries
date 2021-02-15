project(dynamical_systems)

get_target_property(STATE_REPRESENTATION_INCLUDES state_representation INCLUDE_DIRECTORIES)
include_directories(
    include
    ${STATE_REPRESENTATION_INCLUDES}
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
    DESTINATION include)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

if (runtests)
    add_executable(runTestLinear tests/testLinear.cpp)
    target_link_libraries(runTestLinear
      gtest
      gtest_main
      ${PROJECT_NAME}
      state_representation
    )
    add_test(NAME runTestLinear COMMAND runTestLinear)

    add_executable(runTestCircular tests/testCircular.cpp)
    target_link_libraries(runTestCircular
      gtest 
      gtest_main
      ${PROJECT_NAME}
      state_representation
    )
    add_test(NAME runTestCircular COMMAND runTestCircular)
endif()
