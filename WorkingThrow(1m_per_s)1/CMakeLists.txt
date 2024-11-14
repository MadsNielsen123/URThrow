cmake_minimum_required(VERSION 3.5)

project(Futest7)

# Find the ur_rtde library
find_package(ur_rtde REQUIRED)

# Find the Eigen3 library
find_package(Eigen3 REQUIRED)

# Add the executable
add_executable(Futest7 main.cpp)

# Link libraries to the executable
target_link_libraries(Futest7 PRIVATE ur_rtde::rtde Eigen3::Eigen)
