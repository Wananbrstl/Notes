cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
project(useOpenCv)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(test_FAST src/test_FAST.cpp )
target_link_libraries(test_FAST ${OpenCV_LIBS})
