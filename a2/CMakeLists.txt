cmake_minimum_required(VERSION 3.10)
#set(VCPKG_TARGET_TRIPLET "x64-mingw-static" CACHE STRING "" FORCE)
project(Rasterizer)

find_package(OpenCV CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("E:/include/opencv-cpp/opencv/sources/build/install")
include_directories("C:/Program Files (x86)/Eigen3/include")

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)

target_link_libraries(Rasterizer PRIVATE Eigen3::Eigen)
target_link_libraries(Rasterizer PRIVATE ${OpenCV_LIBRARIES})

# -DCMAKE_TOOLCHAIN_FILE=E:\\Program\ Files\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake