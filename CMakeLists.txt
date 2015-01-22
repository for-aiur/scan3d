project(scan3d)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
include_directories( . )
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
	message(FATAL_ERROR "OpenCV required")
else(NOT OpenCV_FOUND)
	message("OpenCV available")
endif(NOT OpenCV_FOUND)
find_package(Boost REQUIRED)
add_library(scan3d STATIC 
						camera.h
						camera.cpp
						dummy_camera.h
                        dummy_camera.cpp
                        calibration_manager.h
                        calibration_manager.cpp
                        calibration_result.h
                        calibration_result.cpp
                        scan_config.h
                        scan_config.cpp
                        scan_manager.h
                        scan_manager.cpp
                        graycode.h
                        graycode.cpp
                        multiview.h
                        multiview.cpp
                        scan_calculator.h
                        scan_calculator.cpp
                        numeric.cpp
                        numeric.h
                        )
target_link_libraries( scan3d ${OpenCV_LIBS})
add_executable(scan3d_test scan3d_test.cpp)
target_link_libraries(scan3d_test scan3d)