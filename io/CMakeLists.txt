cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_library(io OBJECT   
    usbcamera/usbcamera.cpp
)


target_link_libraries(io yaml-cpp pthread)
