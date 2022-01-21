cmake_minimum_required(VERSION 3.1)
project(estimation)

find_package(OpenCV REQUIRED)
find_package(g2o  REQUIRED)
find_package(Sophus REQUIRED)
link_directories(${OpenCV_INCLUDE_DIRS}
                 ${g2o_INCLUDE_DIRS}
                 ${Sophus_INCLUDE_DIRS}
                 "/usr/include/eigen3/")
add_executable(pose_estimation_2d2d src/pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})
#add_executable(pose_estimation_3d2d src/pose_estimation_3d2d.cpp)
#target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS}  ${g2o_LIBS})
add_executable(pose_estimation_3d3d src/pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d ${OpenCV_LIBS}  g2o_core g2o_stuff ${Sophus_LIBS})
