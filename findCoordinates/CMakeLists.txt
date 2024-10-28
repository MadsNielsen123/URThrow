cmake_minimum_required(VERSION 2.8)
project(ballIdentification LANGUAGES CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# OpenCV
find_package( OpenCV REQUIRED )

# Pylon
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon)
include_directories(${Pylon_INCLUDE_DIRS})

add_executable( ballIdentification main.cpp)
target_link_libraries( ballIdentification ${OpenCV_LIBS} ${Pylon_LIBRARIES})


#    list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
#    find_package(Pylon)
#    include_directories(${Pylon_INCLUDE_DIRS})
#    target_link_libraries(MY_TARGET_NAME ${Pylon_LIBRARIES})
