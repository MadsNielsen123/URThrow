cmake_minimum_required(VERSION 3.14)

project(URVisionIntegration LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

# Pylon
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon REQUIRED)
include_directories(${Pylon_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED )

# GRIPPER TCP
find_package(Qt5 REQUIRED COMPONENTS Core Network)

#RTDE
find_package(ur_rtde REQUIRED)

#Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(URVisionIntegration
  main.cpp
  ur5.h ur5.cpp
)
target_link_libraries(URVisionIntegration Qt${QT_VERSION_MAJOR}::Core Qt5::Network ${OpenCV_LIBS}  ${Pylon_LIBRARIES} ur_rtde::rtde Eigen3::Eigen)
