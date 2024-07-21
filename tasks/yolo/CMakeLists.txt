cmake_minimum_required(VERSION 3.16)

find_package(OpenCV REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2023.3.0/runtime/cmake")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(yolo OBJECT
            yolov8.cpp)

target_link_libraries(yolo openvino::runtime ${OpenCV_LIBS})