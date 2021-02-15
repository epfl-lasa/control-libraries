project(robot_model)

find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

get_target_property(STATE_REPRESENTATION_INCLUDES state_representation INCLUDE_DIRECTORIES)
include_directories(
  include
  ${STATE_REPRESENTATION_INCLUDES}
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

if (runtests)
  if (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    add_definitions(-D__GLIBCXX__)
  endif (APPLE)

  add_subdirectory(lib/googletest)

  enable_testing()

  # Include the gtest library. gtest_SOURCE_DIR is available due to
  # 'project(gtest)' above.
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  add_executable(runTestRobotModel tests/testRobotModel.cpp)
  target_link_libraries(runTestRobotModel
    gtest
    gtest_main
    ${PROJECT_NAME}
  )
  add_test(NAME runTestRobotModel COMMAND runTestRobotModel)
endif()
