# Set minimum cmake version
cmake_minimum_required(VERSION 3.5)

# Name project
project(vtx_test)

# Build with debug symbols
set(CMAKE_BUILD_TYPE Debug)

# Add include directory
include_directories(".")

# Add executable
add_executable(vtx_test main.c vtx_tramp.c)

target_compile_definitions(vtx_test PRIVATE SERIAL_PORT_COUNT=1)
target_compile_definitions(vtx_test PRIVATE RESOURCE_SOFT_OFFSET=1)
