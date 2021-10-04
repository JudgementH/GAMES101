cmake_minimum_required (VERSION 2.8.11)
project (a0)

find_package(Eigen3 REQUIRED)
include_directories("C:/Program Files (x86)/Eigen3/include")

add_executable (a0 main.cpp)
