cmake_minimum_required(VERSION 2.8.11)
project(FISHEYE_STEREO)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(undistort undistort.cpp)
target_link_libraries(undistort ${OpenCV_LIBS})
