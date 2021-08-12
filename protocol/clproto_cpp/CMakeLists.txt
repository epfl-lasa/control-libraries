cmake_minimum_required(VERSION 3.9)
project(clproto VERSION 3.1.0)

set(CMAKE_CXX_STANDARD 17)

find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(state_representation REQUIRED)

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
set(PROTOBUF_BINDINGDS_DIR ${PROTOBUF_DIR}/bindings/cpp)

add_custom_target(generate_proto_bindings COMMAND make cpp_bindings
    WORKING_DIRECTORY ${PROTOBUF_DIR}
)

file(GLOB_RECURSE GENERATED_PROTO_BINDINGS "${PROTOBUF_BINDINGDS_DIR}/*.pb.cc" "${PROTOBUF_BINDINGDS_DIR}/*.pb.h")

add_library(${PROJECT_NAME}_bindings ${GENERATED_PROTO_BINDINGS})
target_include_directories(${PROJECT_NAME}_bindings PUBLIC ${PROTOBUF_BINDINGDS_DIR})

add_library(${PROJECT_NAME} src/clproto.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC protobuf state_representation PRIVATE ${PROJECT_NAME}_bindings)


install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
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
