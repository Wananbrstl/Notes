cmake_minimum_required(VERSION 3.2)
project(use_Ceres)

set(CMAKE_BUILD_TYPE Realease)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)
message([python3]:${Python3_INCLUDE_DIRS})
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
include_directories(${CERES_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    )
include_directories("/usr/include/eigen3" ${MATPLOTLIB_CPP_INCLUDE_DIRS})


add_executable(demo01 src/demo01.cpp)
target_link_libraries(demo01 ${CERES_LIBRARIES} ${OpenCV_LIBS} )
add_executable(demo03 src/demo03.cpp)
target_link_libraries(demo03 ${CERES_LIBRARIES} ${OpenCV_LIBS} )
add_executable(demo04 src/demo04.cpp)
target_link_libraries(demo04 ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} ${MATPLOTLIB_CPP_INCLUDE_DIRS} Python3::Python Python3::NumPy )
target_include_directories(demo04 PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

add_executable(gauss_newton src/gauss_newton.cpp)
target_link_libraries(gauss_newton ${CERES_LIBRARIES} ${OpenCV_LIBS}  ${MATPLOTLIB_CPP_INCLUDE_DIRS} Python3::Python Python3::NumPy)
target_include_directories(gauss_newton PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

