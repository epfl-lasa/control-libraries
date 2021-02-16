project(controllers)

get_target_property(STATE_REPRESENTATION_INCLUDES state_representation INCLUDE_DIRECTORIES)
include_directories(
    include
    ${STATE_REPRESENTATION_INCLUDES}
)

set(CORE_SOURCES
  src/impedance/Impedance.cpp
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
    add_executable(runTestImpedanceController tests/testImpedanceController.cpp)
    target_link_libraries(runTestImpedanceController
      gtest
      gtest_main
      ${PROJECT_NAME}
      state_representation
    )
    add_test(NAME runTestImpedanceController COMMAND runTestImpedanceController)
endif()
