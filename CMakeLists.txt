project(scan3d)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
include_directories( /usr/local/include )
link_directories( /usr/local/lib )
target_link_libraries( opencv_core )
add_library(scan3d STATIC camera.cpp dummy_camera.cpp)


