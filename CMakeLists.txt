

# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(kuksa-lane-assist VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# Generate protobuf stubs
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

# Add the executable
add_executable(kuksa-lane-assist src/main.cpp 
                ${PROTO_SRCS} ${PROTO_HDRS})

# Add all the dependencies
target_link_libraries(kuksa-lane-assist
                        ${Protobuf_LIBRARIES} grpc grpc++ kuksaclient protobuf)
target_include_directories(kuksa-lane-assist PUBLIC  include proto)