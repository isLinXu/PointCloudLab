cmake_minimum_required(VERSION 3.15)
project(PointCloudLab)

set(CMAKE_CXX_STANDARD 14)

# --------------------------------相关视觉库---------------------------------
# opencv
set(TARGET_LIBS "/usr/local/include/opencv4/")
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_features2d opencv_photo opencv_video opencv_objdetect)


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PointCloudLab main.cpp
        )

#目标链接库
target_link_libraries( PointCloudLab
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
        )


