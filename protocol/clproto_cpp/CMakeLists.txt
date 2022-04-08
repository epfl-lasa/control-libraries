cmake_minimum_required(VERSION 3.9)
project(clproto VERSION 5.1.6)

set(CMAKE_CXX_STANDARD 17)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_libraries ${PROJECT_VERSION} REQUIRED state_representation)

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

set(PROTOBUF_DIR ${PROJECT_SOURCE_DIR}/../protobuf)
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
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER ${PROJECT_SOURCE_DIR}/include/clproto.h)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY} state_representation PRIVATE ${PROJECT_NAME}_bindings)

install(TARGETS ${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION include
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

if (BUILD_TESTING)
    file(GLOB_RECURSE TEST_SOURCES test/ test_*.cpp)
    add_executable(test_${PROJECT_NAME} ${TEST_SOURCES})
    target_link_libraries(test_${PROJECT_NAME}
        protobuf
        ${PROJECT_NAME}
        state_representation
        ${GTEST_LIBRARIES}
        pthread
    )
    add_test(NAME test_${PROJECT_NAME} COMMAND test_${PROJECT_NAME})
endif ()
