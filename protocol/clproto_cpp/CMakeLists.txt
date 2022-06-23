cmake_minimum_required(VERSION 3.15)

project(clproto VERSION 6.0.2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

include(FindProtobuf)
find_package(Protobuf 3.17 REQUIRED)
find_package(control_libraries ${PROJECT_VERSION} REQUIRED COMPONENTS state_representation)

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  if (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    add_definitions(-D__GLIBCXX__)
  endif (APPLE)
else()
  find_package(GTest QUIET)
endif()

set(PROTOBUF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../protobuf)
set(PROTOBUF_BINDINGS_DIR ${PROTOBUF_DIR}/bindings/cpp)

add_custom_target(generate_proto_bindings COMMAND make cpp_bindings
  WORKING_DIRECTORY ${PROTOBUF_DIR}
)

file(GLOB_RECURSE GENERATED_PROTO_BINDINGS "${PROTOBUF_BINDINGS_DIR}/*.pb.cc" "${PROTOBUF_BINDINGS_DIR}/*.pb.h")

add_library(${PROJECT_NAME}_bindings STATIC ${GENERATED_PROTO_BINDINGS})
target_include_directories(${PROJECT_NAME}_bindings PUBLIC ${PROTOBUF_BINDINGS_DIR})
set_property(TARGET ${PROJECT_NAME}_bindings PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE_DIR}/src/clproto.cpp
  ${PROJECT_SOURCE_DIR}/src/decoders.cpp
  ${PROJECT_SOURCE_DIR}/src/encoders.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/clproto.h)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY} state_representation PRIVATE ${PROJECT_NAME}_bindings)

# install the target and create export-set
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS ${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# generate and install export file
install(EXPORT ${PROJECT_NAME}_targets
  FILE ${PROJECT_NAME}_targets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if (BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/ test_*.cpp)
  add_executable(test_${PROJECT_NAME} ${TEST_SOURCES})
  target_link_libraries(test_${PROJECT_NAME}
    protobuf
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME test_${PROJECT_NAME} COMMAND test_${PROJECT_NAME})
endif ()

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# install config files
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
