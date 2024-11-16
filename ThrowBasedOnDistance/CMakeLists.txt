cmake_minimum_required(VERSION 3.5)

project(Futest16)

# Find the ur_rtde library
find_package(ur_rtde REQUIRED)

# Find the Eigen3 library
find_package(Eigen3 REQUIRED)

# Add the executable
add_executable(Futest16 main.cpp)

# Link libraries to the executable
target_link_libraries(Futest16 PRIVATE ur_rtde::rtde Eigen3::Eigen)
