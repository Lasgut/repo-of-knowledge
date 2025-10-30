# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(NetworkingProject VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${CMAKE_SOURCE_DIR}/sources)

# Test  executable
set(UDP_TEST_SOURCES
    MainUdpTest.cpp
    sources/UdpBase.cpp
    sources/UdpClient.cpp
    sources/UdpServer.cpp
)
add_executable(udp_test ${UDP_TEST_SOURCES})

# Server executable
set(UDP_SERVER_SOURCES
    MainUdpServer.cpp
    sources/UdpBase.cpp
    sources/UdpServer.cpp
)
add_executable(udp_server ${UDP_SERVER_SOURCES})

# Client executable
set(UDP_CLIENT_SOURCES
    MainUdpClient.cpp
    sources/UdpBase.cpp
    sources/UdpClient.cpp
)
add_executable(udp_client ${UDP_CLIENT_SOURCES})

# Node executable
set(UDP_NODE_SOURCES
    MainUdpNode.cpp
    sources/UdpBase.cpp
    sources/UdpClient.cpp
    sources/UdpServer.cpp
)
add_executable(udp_node ${UDP_NODE_SOURCES})

# Link libraries if needed (e.g., pthread for networking)
find_package(Threads REQUIRED)
target_link_libraries(udp_test Threads::Threads)
target_link_libraries(udp_server Threads::Threads)
target_link_libraries(udp_client Threads::Threads)
target_link_libraries(udp_node Threads::Threads)