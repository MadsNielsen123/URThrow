cmake_minimum_required(VERSION 3.5)

project(calibration2 LANGUAGES CXX)

find_package( OpenCV REQUIRED )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(calibration2 main.cpp)

include(GNUInstallDirs)
install(TARGETS calibration2
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

target_link_libraries(calibration2 ${OpenCV_LIBS} )
