set(LIBRARY_NAME robot_model)

set(PINOCCHIO_VERSION 2.6.9)
set(OSQP_EIGEN_VERSION 0.6.4)
find_package(pinocchio ${PINOCCHIO_VERSION} REQUIRED)
find_package(OsqpEigen ${OSQP_EIGEN_VERSION} REQUIRED)

set(CORE_SOURCES
  src/Model.cpp
)

add_library(${LIBRARY_NAME} SHARED ${CORE_SOURCES})
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# add include directories
target_include_directories(${LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${LIBRARY_NAME}
  state_representation
  ${PINOCCHIO_LIBRARIES}
  OsqpEigen::OsqpEigen
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
  add_executable(test_${LIBRARY_NAME} test/test_robot_model.cpp)
  file(GLOB_RECURSE MODULE_TEST_SOURCES test/tests test_*.cpp)
  target_sources(test_${LIBRARY_NAME} PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(test_${LIBRARY_NAME}
    ${LIBRARY_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  target_compile_definitions(test_${LIBRARY_NAME} PRIVATE TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/")
  add_test(NAME test_${LIBRARY_NAME} COMMAND test_${LIBRARY_NAME})
endif ()

if(${PKG_CONFIG_FOUND})
  set(PKG_NAME ${LIBRARY_NAME})
  set(PKG_DESC "This library is a wrapper for the pinocchio library, used to compute dynamics model of a robot.")
  set(pkg_conf_file "robot_model.pc")
  set(PKG_EXTERNAL_DEPS "state_representation >= ${CMAKE_PROJECT_VERSION}, pinocchio >= ${PINOCCHIO_VERSION}")
  set(PKG_CFLAGS "-lOsqpEigen")
  configure_file("${CMAKE_SOURCE_DIR}/library_template.pc.in" "${CMAKE_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES "${CMAKE_BINARY_DIR}/${pkg_conf_file}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
endif()
