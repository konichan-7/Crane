cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(io OBJECT   
    usbcamera/usbcamera.cpp
    cboard/cboard.cpp
)


target_link_libraries(io yaml-cpp pthread ${OpenCV_LIBS})
