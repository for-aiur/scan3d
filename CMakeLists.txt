project(scan3d)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
include_directories( /usr/local/include )
include_directories( . )
link_directories( /usr/local/lib )
find_package(OpenCV REQUIRED core highgui calib3d flann)
find_package(Boost REQUIRED)
add_library(scan3d STATIC camera.cpp
                        dummy_camera.cpp
                        calibration_manager.cpp
                        calibration_result.cpp
                        scan_config.cpp
                        scan_manager.cpp
                        graycode.cpp
                        multiview.cpp
                        scan_calculator.cpp
                        )
target_link_libraries( scan3d opencv_core opencv_highgui opencv_flann opencv_calib3d)
