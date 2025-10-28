# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(NetworkingProject VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add source files
set(UDP_TEST_SOURCES
    BasicUdpTest.cpp
    sources/UdpClient.cpp
    sources/UdpServer.cpp
)

# Add include directories
include_directories(include)

# Create the executable
add_executable(udp_test ${UDP_TEST_SOURCES})

set(UDP_SOURCES
    UdpSendReceive.cpp
    sources/UdpClient.cpp
    sources/UdpServer.cpp
)

add_executable(${PROJECT_NAME} ${UDP_SOURCES})

# Link libraries if needed (e.g., pthread for networking)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} Threads::Threads)