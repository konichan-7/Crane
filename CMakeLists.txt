cmake_minimum_required(VERSION 3.16.3)

project(crane)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/yolo)

add_executable(capture examples/capture.cpp)
target_link_libraries(capture ${OpenCV_LIBS} io fmt::fmt tools yaml-cpp)

add_executable(weights_detect examples/weights_detect.cpp)
target_link_libraries(weights_detect ${OpenCV_LIBS} fmt::fmt yaml-cpp  tools io)

add_executable(crane src/crane.cpp)
target_link_libraries(crane ${OpenCV_LIBS} fmt::fmt yaml-cpp yolo tools io)
