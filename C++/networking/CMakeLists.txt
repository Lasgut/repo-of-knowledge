cmake_minimum_required(VERSION 3.10)

project(NetworkingProject VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Collect all sources recursively from the sources/ directory
# CONFIGURE_DEPENDS makes CMake re-evaluate when files are added/removed
file(GLOB_RECURSE UDP_SOURCES
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/sources/*.cpp
)

if(NOT UDP_SOURCES)
    message(WARNING "No sources found under ${PROJECT_SOURCE_DIR}/sources")
endif()

# Create a library from all sources so multiple targets can link to it
add_library(udp_lib STATIC ${UDP_SOURCES})

# Make headers in sources/ visible to consumers of udp_lib
target_include_directories(udp_lib PUBLIC ${PROJECT_SOURCE_DIR}/sources/interface)
target_include_directories(udp_lib PUBLIC ${PROJECT_SOURCE_DIR}/sources/udp)
target_include_directories(udp_lib PUBLIC ${PROJECT_SOURCE_DIR}/sources/utilities)

# Make sure we have threads for networking code that needs it
find_package(Threads REQUIRED)
target_link_libraries(udp_lib PUBLIC Threads::Threads)

# Add executables that only contain their main files and link the common library
add_executable(udp_test MainUdpTest.cpp)
target_link_libraries(udp_test PRIVATE udp_lib)

add_executable(udp_server MainUdpServer.cpp)
target_link_libraries(udp_server PRIVATE udp_lib)

add_executable(udp_client MainUdpClient.cpp)
target_link_libraries(udp_client PRIVATE udp_lib)

add_executable(udp_node MainUdpNode.cpp)
target_link_libraries(udp_node PRIVATE udp_lib)