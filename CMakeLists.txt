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
add_subdirectory(tasks/auto_crane)

add_executable(cboard_rotate_test examples/cboard_rotate_test.cpp)
target_link_libraries(cboard_rotate_test ${OpenCV_LIBS} fmt::fmt io tools)

add_executable(cboard_rx_test examples/cboard_rx_test.cpp)
target_link_libraries(cboard_rx_test ${OpenCV_LIBS} fmt::fmt io tools)

add_executable(cboard_tx_test examples/cboard_tx_test.cpp)
target_link_libraries(cboard_tx_test ${OpenCV_LIBS} fmt::fmt io tools)

add_executable(usbcamera_test examples/usbcamera_test.cpp)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt io tools)

add_executable(solver_test examples/solver_test.cpp)
target_link_libraries(solver_test ${OpenCV_LIBS} fmt::fmt io tools auto_crane)

add_executable(matcher_test examples/matcher_test.cpp)
target_link_libraries(matcher_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_crane tools io)

add_executable(main src/crane.cpp src/main.cpp)
target_link_libraries(main ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_crane tools io)