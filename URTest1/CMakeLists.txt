cmake_minimum_required(VERSION 3.5)

project(URTest1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ur_rtde REQUIRED)

add_executable(URTest1 main.cpp)

target_link_libraries(URTest1 PRIVATE ur_rtde::rtde)
