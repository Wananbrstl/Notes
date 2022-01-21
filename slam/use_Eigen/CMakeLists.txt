cmake_minimum_required(VERSION 3.8)
project(use_Eigen)

include_directories("/usr/include/eigen3/")
add_executable(useEigen src/useEigen.cpp)
