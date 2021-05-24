cmake_minimum_required(VERSION 3.9)
project(control_libraries_protocol)

set(CMAKE_CXX_STANDARD 17)

find_package(Protobuf REQUIRED)

find_package(Eigen3 REQUIRED)
find_library(state_representation REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS protobuf/cartesian_state.proto)
message(${PROTO_SRCS})
message(${PROTO_HDRS})

add_executable(${PROJECT_NAME} main.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} PUBLIC protobuf state_representation)