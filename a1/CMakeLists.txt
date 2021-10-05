cmake_minimum_required(VERSION 3.10)
project(a1)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("E:/include/opencv-cpp/opencv/sources/build/install")
include_directories("C:/Program Files (x86)/Eigen3/include")

add_executable(a1 main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(a1 ${OpenCV_LIBRARIES})
