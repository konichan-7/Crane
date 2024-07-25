cmake_minimum_required(VERSION 3.16)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2023.3.0/runtime/cmake")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

include_directories(${EIGEN3_INCLUDE_DIR})

add_library(auto_crane OBJECT
            yolov8.cpp
            decider.cpp
            matcher.cpp)

target_link_libraries(auto_crane openvino::runtime ${OpenCV_LIBS} fmt::fmt)