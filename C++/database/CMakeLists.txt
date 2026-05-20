cmake_minimum_required(VERSION 3.10)

project(DatabaseProject VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(database_lib INTERFACE)

target_include_directories(database_lib INTERFACE
    ${PROJECT_SOURCE_DIR}/sources
)

find_package(Threads REQUIRED)
target_link_libraries(database_lib INTERFACE Threads::Threads)

add_executable(database_test MainDatabaseTest.cpp)
target_link_libraries(database_test PRIVATE database_lib)