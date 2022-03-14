cmake_minimum_required(VERSION 3.15)

project(control_libraries VERSION 5.0.10)

# Build options
option(BUILD_TESTING "Build all tests." OFF)
option(BUILD_CONTROLLERS "Build and install controllers library" ON)
option(BUILD_DYNAMICAL_SYSTEMS "Build and install dynamical systems library" ON)
option(BUILD_ROBOT_MODEL "Build and install robot model library" ON)
option(EXPERIMENTAL_FEATURES "Include experimental features" OFF)

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

find_package(Eigen3 REQUIRED)

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

add_subdirectory(state_representation)
list(APPEND INSTALL_INTERFACE_LINK_LIBRARIES state_representation)
get_target_property(TARGET_INTERFACE_LINK_LIBRARIES state_representation INTERFACE_LINK_LIBRARIES)
if(TARGET_INTERFACE_LINK_LIBRARIES)
  list(APPEND INSTALL_INTERFACE_LINK_LIBRARIES "${TARGET_INTERFACE_LINK_LIBRARIES}")
endif()

if(BUILD_DYNAMICAL_SYSTEMS)
  add_subdirectory(dynamical_systems)
  list(APPEND INSTALL_INTERFACE_LINK_LIBRARIES dynamical_systems)
  get_target_property(TARGET_INTERFACE_LINK_LIBRARIES dynamical_systems INTERFACE_LINK_LIBRARIES)
  if(TARGET_INTERFACE_LINK_LIBRARIES)
    list(APPEND INSTALL_INTERFACE_LINK_LIBRARIES "${TARGET_INTERFACE_LINK_LIBRARIES}")
  endif()
endif()

if(BUILD_ROBOT_MODEL OR BUILD_CONTROLLERS)
  add_subdirectory(robot_model)
  list(APPEND INSTALL_INTERFACE_LINK_LIBRARIES robot_model)
  get_target_property(TARGET_INTERFACE_LINK_LIBRARIES robot_model INTERFACE_LINK_LIBRARIES)
  if(TARGET_INTERFACE_LINK_LIBRARIES)
    list(APPEND INSTALL_INTERFACE_LINK_LIBRARIES "${TARGET_INTERFACE_LINK_LIBRARIES}")
  endif()
endif()

if(BUILD_CONTROLLERS)
  add_subdirectory(controllers)
  list(APPEND INSTALL_INTERFACE_LINK_LIBRARIES controllers)
  get_target_property(TARGET_INTERFACE_LINK_LIBRARIES controllers INTERFACE_LINK_LIBRARIES)
  if(TARGET_INTERFACE_LINK_LIBRARIES)
    list(APPEND INSTALL_INTERFACE_LINK_LIBRARIES "${TARGET_INTERFACE_LINK_LIBRARIES}")
  endif()
endif()

list(REMOVE_DUPLICATES INSTALL_INTERFACE_LINK_LIBRARIES)

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
